// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file core/energy_methods/MgEnergy.cc
/// @brief
/// @details
/// @author Rhiju Das, rhiju@stanford.edu


#include <core/energy_methods/MgEnergy.hh>
#include <core/energy_methods/MgEnergyCreator.hh>
#include <core/scoring/magnesium/MgKnowledgeBasedPotential.hh>
#include <core/scoring/magnesium/util.hh>

// Package headers
#include <core/scoring/MinimizationData.hh>
#include <core/scoring/ResidueNeighborList.hh>
#include <core/scoring/DerivVectorPair.hh>
#include <core/scoring/etable/count_pair/CountPairAll.hh>
#include <core/scoring/hbonds/HBondOptions.hh>
#include <core/scoring/hbonds/hbonds_geom.hh>
#include <core/scoring/func/FadeFunc.hh>

// Project headers
#include <core/pose/Pose.hh>
#include <core/conformation/Residue.hh>
#include <basic/options/option.hh>
#include <basic/options/keys/score.OptionKeys.gen.hh>
#include <utility/vector1.hh>
#include <numeric/deriv/angle_deriv.hh>

#include <core/scoring/EnergyMap.hh> // AUTO IWYU For EMapVector

using namespace core::chemical::rna;
using namespace basic::options;
using namespace basic::options::OptionKeys;

#include <basic/Tracer.hh>

static basic::Tracer TR( "core.energy_methods.MgEnergy" );

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Enforces Mg(2+) to have 6 octahedrally coordinated ligands.
//
// Octahedral axes ('orbital frame' or 'ligand field') defined by
//  perpendicular virtual atoms V1, V2, V3, V4, V5, V6:
//
//        V2 V6
//         |/
//   V4 -- Mg -- V1
//        /|
//      V3 V5
//
// Basic interaction potential mg_lig is defined in terms of three geometric parameters:
//
//                 Base
//                 /
//   Mg -- V   :Acc
//
//   1.  Dist(  Mg -- Acc )         [should be near 2.1 Angstroms]
//   2.  Angle( Acc -- Mg -- V)     [should be near 0.0; cos angle should be near +1.0]
//   3.  Angle( Mg -- Acc -- Base ) [should be near 120-180 degrees; cos angle should be < -0.5]
//
// Also include terms:
//
//   mg_sol  [penalty for blocking fluid water]
//   mg_ref  [cost of instantiating mg(2+); put into ref?]
//   core::scoring::hoh_ref [cost of instantiating water]
//
//              -- rhiju, 2015
//
// Note: for cost of instantiating water, could instead use:
//
//    h2o_intra, [in WaterAdductIntraEnergyCreator -- check if activated]
// OR pointwater [when Frank's PWAT is checked in from branch dimaio/waterstuff.]
//
// will need to make a decision when dust settles on HOH.
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////


namespace core {
namespace energy_methods {

/// @details This must return a fresh instance of the MgEnergy class,
/// never an instance already in use
core::scoring::methods::EnergyMethodOP
MgEnergyCreator::create_energy_method(
	core::scoring::methods::EnergyMethodOptions const &
) const {
	return utility::pointer::make_shared< MgEnergy >();
}

core::scoring::ScoreTypes
MgEnergyCreator::score_types_for_method() const {
	using namespace core::scoring;
	ScoreTypes sts;
	sts.push_back( mg );
	sts.push_back( mg_lig );
	sts.push_back( mg_sol );
	sts.push_back( mg_ref );
	sts.push_back( core::scoring::hoh_ref );
	return sts;
}


MgEnergy::MgEnergy() :
	parent( utility::pointer::make_shared< MgEnergyCreator >() ),
	// following are for mg_lig term.
	mg_lig_knowledge_based_potential_( utility::pointer::make_shared< core::scoring::magnesium::MgKnowledgeBasedPotential >() ),
	mg_lig_interaction_cutoff_( 4.0 ),
	v_angle_width_( mg_lig_knowledge_based_potential_->v_angle_width() ),
	v_angle_width2_( v_angle_width_ * v_angle_width_ ),
	v_angle_baseline( 0.3 ), // arbitrary -- will need to be optimized
	// following are ref terms -- again arbitrary for now.
	mg_ref_score_( 15.0 ), // counteracts 'self' energy (Mg--water); and Mg freeze-out penalty
	hoh_ref_score_( 1.0 ),
	// Following are for solvation. "Wild guesses" from database/chemical/fa_standard/atom_properties.txt --
	// don't use those directly so that we can play with them separately.
	mg_lj_radius_( 1.185 ),
	mg_lk_lambda_( 4.500 ), // 3.5 for other atoms -- but this is charged.
	mg_lk_dgfree_( -200.00 ),
	lk_inv_lambda2( 1.0 / (mg_lk_lambda_ * mg_lk_lambda_) ),
	inv_neg2_tms_pi_sqrt_pi( -0.089793561062583294 ),
	mg_lk_coeff( inv_neg2_tms_pi_sqrt_pi * mg_lk_dgfree_ / mg_lk_lambda_ ),
	compute_mg_sol_for_hydrogens_( option[ score::compute_mg_sol_for_hydrogens ]() ),
	// fading solvation
	mg_sol_interaction_cutoff_( 6.0 ),
	mg_sol_fade_zone_( 0.1 ), // turn off mg_sol smoothly between 5.9 and 6.0.
	mg_sol_fade_func_( utility::pointer::make_shared< core::scoring::func::FadeFunc >( -10.0, mg_sol_interaction_cutoff_, mg_sol_fade_zone_, 1.0 ) )
{}


/// clone
core::scoring::methods::EnergyMethodOP
MgEnergy::clone() const
{
	return utility::pointer::make_shared< MgEnergy >();
}

void
MgEnergy::setup_for_scoring( pose::Pose & pose, core::scoring::ScoreFunction const & ) const
{
	pose.update_residue_neighbors();
}

/////////////////////////////////////////////////////////////////////////////
// scoring
/////////////////////////////////////////////////////////////////////////////
void
MgEnergy::residue_pair_energy(
	conformation::Residue const & rsd1,
	conformation::Residue const & rsd2,
	pose::Pose const & pose,
	core::scoring::ScoreFunction const &,
	core::scoring::EnergyMap & emap
) const
{
	if ( rsd2.name3() == " MG" ) {
		residue_pair_energy_one_way( rsd1, rsd2, pose, emap );
	} else if ( rsd1.name3() == " MG" ) {
		residue_pair_energy_one_way( rsd2, rsd1, pose, emap );
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void
MgEnergy::residue_pair_energy_one_way(
	conformation::Residue const & rsd1, // The ligand residue
	conformation::Residue const & rsd2, // The Mg(2+)
	pose::Pose const & pose,
	core::scoring::EnergyMap & emap
) const {

	core::scoring::EnergyMap weights; // empty, would be used for derivs.
	utility::vector1< core::scoring::DerivVectorPair > r1_atom_derivs, r2_atom_derivs; // empty, would be used for derivs.

	// Loop over potential ligand positions.
	// using same eval_mg_interaction() function as residue_pair_ext to avoid copying code.
	for ( Size i = 1; i <= rsd1.natoms(); i++ ) {
		eval_mg_interaction( rsd1, i, rsd2, pose, emap, weights, r1_atom_derivs, r2_atom_derivs );
	}
}



/////////////////////////////////
void
MgEnergy::residue_pair_energy_ext(
	conformation::Residue const & ires,
	conformation::Residue const & jres,
	core::scoring::ResPairMinimizationData const & min_data,
	pose::Pose const & pose,
	core::scoring::ScoreFunction const &,
	core::scoring::EnergyMap & emap
) const
{
	core::scoring::EnergyMap weights; // empty, would be used for derivs.
	utility::vector1< core::scoring::DerivVectorPair > r1_atom_derivs, r2_atom_derivs; // empty, would be used for derivs.
	eval_residue_pair( ires, jres, min_data, pose, emap, weights, r1_atom_derivs, r2_atom_derivs );
}

/////////////////////////////////
void
MgEnergy::eval_intrares_energy(
	conformation::Residue const & rsd,
	pose::Pose const &,
	core::scoring::ScoreFunction const &,
	core::scoring::EnergyMap & emap
) const {
	if ( rsd.name3() == " MG" ) emap[ core::scoring::mg_ref  ] += mg_ref_score_;
	if ( rsd.aa() == core::chemical::aa_h2o ) {
		if ( !rsd.has_variant_type( core::chemical::VIRTUAL_RESIDUE_VARIANT ) ) emap[ core::scoring::hoh_ref ] += hoh_ref_score_;
	}
}

////////////////////////////////////////////////////
void
MgEnergy::eval_residue_pair_derivatives(
	conformation::Residue const & ires,
	conformation::Residue const & jres,
	core::scoring::ResSingleMinimizationData const &,
	core::scoring::ResSingleMinimizationData const &,
	core::scoring::ResPairMinimizationData const & min_data,
	pose::Pose const & pose, // provides context
	core::scoring::EnergyMap const & weights,
	utility::vector1< core::scoring::DerivVectorPair > & r1_atom_derivs,
	utility::vector1< core::scoring::DerivVectorPair > & r2_atom_derivs) const
{
	core::scoring::EnergyMap emap; // dummy -- will not be used.
	eval_residue_pair( ires, jres, min_data, pose, emap, weights, r1_atom_derivs, r2_atom_derivs );
}

////////////////////////////////////////////////////
void
MgEnergy::eval_residue_pair(
	conformation::Residue const & ires,
	conformation::Residue const & jres,
	core::scoring::ResPairMinimizationData const & min_data,
	pose::Pose const & pose, // provides context
	core::scoring::EnergyMap & emap, // fill score values in here.
	core::scoring::EnergyMap const & weights, // for derivs.
	utility::vector1< core::scoring::DerivVectorPair > & r1_atom_derivs,
	utility::vector1< core::scoring::DerivVectorPair > & r2_atom_derivs) const
{
	auto const & nblist( static_cast< core::scoring::ResiduePairNeighborList const & > ( min_data.get_data_ref( core::scoring::mg_pair_nblist ) ) );
	utility::vector1< core::scoring::SmallAtNb > const & neighbs( nblist.atom_neighbors() );
	for ( Size k = 1, kend = neighbs.size(); k <= kend; ++k ) {
		Size const ii = neighbs[ k ].atomno1();
		Size const jj = neighbs[ k ].atomno2();
		// NOTE -- if we remove heavyatom constraint, could avoid hydrogens pointing into Mg(2+).
		if ( jres.atom_name( jj ) == "MG  " && ( compute_mg_sol_for_hydrogens_ || ii <= ires.nheavyatoms() ) ) {
			eval_mg_interaction( ires, ii, jres, pose, emap, weights, r1_atom_derivs, r2_atom_derivs );
		} else if ( ires.atom_name( ii ) == "MG  " && ( compute_mg_sol_for_hydrogens_ || jj <= jres.nheavyatoms() ) ) {
			eval_mg_interaction( jres, jj, ires, pose, emap, weights, r2_atom_derivs, r1_atom_derivs );
		}
	}
}

///////////////////////////////////////////////////////////////
void
MgEnergy::eval_mg_interaction(
	conformation::Residue const & rsd1 /* other residue */,
	Size const atomno1                 /* other atomno */,
	conformation::Residue const & rsd2 /* mg residue */,
	pose::Pose const &, // provides context
	core::scoring::EnergyMap & emap,
	core::scoring::EnergyMap const & weights,
	utility::vector1< core::scoring::DerivVectorPair > & r1_atom_derivs /* other residue */,
	utility::vector1< core::scoring::DerivVectorPair > & r2_atom_derivs /* mg residue */
) const {
	using namespace numeric::deriv;

	// get magnesium position
	Size const & i( atomno1 );
	Size const j = 1;  //First atom of Mg2+ residue is assumed to be Mg2+ atom.
	runtime_assert( rsd2.atom_name( j ) ==  "MG  " );

	if ( rsd1.is_virtual( i ) ) return;
	if ( rsd2.is_virtual( j ) ) return;
	if ( !compute_mg_sol_for_hydrogens_ && i > rsd1.nheavyatoms() ) return;

	Vector const & i_xyz( rsd1.xyz( i ) );
	Vector const & j_xyz( rsd2.xyz( j ) );

	Distance d = ( i_xyz - j_xyz ).length();

	// note that hard cutoff may lead to minimization problems --
	// may want to fade solvation term.
	if ( d > mg_sol_interaction_cutoff_  ) return;

	// solvation -- mimic laziridis-karplus form.
	// See core/scoring/etables/Etable.cc
	// could also base this off mg_lig term for HOH (in the spirit of how geom_sol is based on hbond)
	Real const inv_dis = 1.0/d;
	Real const inv_dis2 = inv_dis * inv_dis;
	Real const dis_rad1 = ( d - mg_lj_radius_ );
	Real const x1 = ( dis_rad1 * dis_rad1 ) * lk_inv_lambda2;
	Real const mg_sol_value = std::exp(-x1) * mg_lk_coeff * inv_dis2;
	Real const fade_factor  = mg_sol_fade_func_->func( d );
	Real const mg_sol_score = mg_sol_value * fade_factor;
	emap[ core::scoring::mg     ] += mg_sol_score;
	emap[ core::scoring::mg_sol ] += mg_sol_score;

	if ( r1_atom_derivs.size() > 0 ) { // compute derivatives
		Real const weight = weights[ core::scoring::mg ] + weights[ core::scoring::mg_sol ];
		Real dist_deriv =  ( -2.0 * dis_rad1 * lk_inv_lambda2 - 2.0 * inv_dis ) * mg_sol_value;
		Real fade_deriv = mg_sol_fade_func_->dfunc( d );
		Real const dE_ddist = ( dist_deriv * fade_factor + mg_sol_value * fade_deriv ) * weight;
		Vector f2 =   +1.0 * ( j_xyz - i_xyz ).normalized();
		Vector f1 =    1.0 * cross( f2, j_xyz );
		// acceptor atom
		r1_atom_derivs[ i ].f1() -= dE_ddist * f1;
		r1_atom_derivs[ i ].f2() -= dE_ddist * f2;

		// Mg atom
		r2_atom_derivs[ j ].f1() += dE_ddist * f1;
		r2_atom_derivs[ j ].f2() += dE_ddist * f2;
	}

	if ( !rsd1.heavyatom_is_an_acceptor( i ) ) return;

	if ( d >= mg_lig_interaction_cutoff_  ) return;

	////////////////////
	// Term 1: distance
	////////////////////
	GaussianParameter const & mg_potential_gaussian_parameter = mg_lig_knowledge_based_potential_->get_mg_potential_gaussian_parameter( rsd1, i );
	runtime_assert( mg_potential_gaussian_parameter.center > 0.0 ); // should be defined for all acceptors.
	Real dist_score = core::scoring::magnesium::get_gaussian_potential_score( mg_potential_gaussian_parameter, i_xyz, j_xyz );

	//////////////////////////////////////////////////////////////////
	// Term 2: form factor for angle Mg -- Acceptor -- Acceptor-Base
	//////////////////////////////////////////////////////////////////
	GaussianParameter const & mg_potential_costheta_gaussian_parameter = mg_lig_knowledge_based_potential_->get_mg_potential_costheta_gaussian_parameter( rsd1, i );
	Real acc_angle_form_factor( 1.0 ), acc_angle_form_factor_OH1( 1.0 ), acc_angle_form_factor_OH2( 1.0 );
	Real cos_theta( 0.0 ), cos_theta_OH1( 0.0 ), cos_theta_OH2( 0.0 );
	Size i_base( 0 ), OH1( 2 ), OH2( 3 );
	Vector base_xyz( 0.0 );
	bool const is_water( rsd1.name3() == "HOH" );
	if ( is_water ) { // treat both H's as base atoms, symmetrically
		cos_theta_OH1     = core::scoring::magnesium::get_cos_theta( rsd1, i, j_xyz, OH1 );
		cos_theta_OH2 = core::scoring::magnesium::get_cos_theta( rsd1, i, j_xyz, OH2 );
		acc_angle_form_factor_OH1 = core::scoring::magnesium::get_gaussian_score( mg_potential_costheta_gaussian_parameter, cos_theta_OH1 );
		acc_angle_form_factor_OH2 = core::scoring::magnesium::get_gaussian_score( mg_potential_costheta_gaussian_parameter, cos_theta_OH2 );
		acc_angle_form_factor = 0.5 * ( acc_angle_form_factor_OH1 + acc_angle_form_factor_OH2);
	} else {
		cos_theta = core::scoring::magnesium::get_cos_theta( rsd1, i, j_xyz, i_base, base_xyz );
		acc_angle_form_factor = core::scoring::magnesium::get_gaussian_score( mg_potential_costheta_gaussian_parameter, cos_theta );
	}

	//////////////////////////////////////////////////////////////////
	// Term 3: form factor for angle Acceptor -- Mg -- V
	//////////////////////////////////////////////////////////////////
	Real const cos_v_angle = core::scoring::magnesium::get_cos_angle_to_closest_orbital_axis( rsd2, i_xyz );
	Real const v_angle_form_factor = exp( - ( 1.0 - cos_v_angle ) / (2.0 * v_angle_width2_ ) ); // funny functional form ~ (v_angle/2 width)^2
	Real const v_angle_form_factor_faded = v_angle_baseline + ( 1.0 - v_angle_baseline ) * v_angle_form_factor; // unity if perfect angle, v_angle_baseline if not.

	// Note: treated as a product -- not quite consistent with derivation from log-stats.
	// [ could instead add as sum, and then do fading on potential near boundaries, as in hbonds. ]
	Real const mg_lig_score = dist_score * acc_angle_form_factor * v_angle_form_factor_faded;

	//  TR << "MG_LIG_SCORE: from " <<  rsd2.seqpos()  << " to "
	//    << rsd1.seqpos()  << " " << rsd1.name3() << " " << rsd1.atom_name( atomno1 ) << " ==> " << mg_lig_score << "   (dist) " << dist_score << "   (acc_angle) " << acc_angle_form_factor << "  (v_angle) " << v_angle_form_factor_faded << std::endl;

	emap[ core::scoring::mg ]     += mg_lig_score;
	emap[ core::scoring::mg_lig ] += mg_lig_score;

	/////////////////
	// Derivatives
	/////////////////
	if ( r1_atom_derivs.size() <= 0 ) return;

	Real const weight = weights[ core::scoring::mg ] + weights[ core::scoring::mg_lig ];

	////////////////////
	// Term 1: distance
	////////////////////
	Real dist_deriv = core::scoring::magnesium::get_gaussian_deriv( mg_potential_gaussian_parameter, d );
	Real const dE_ddist = dist_deriv * acc_angle_form_factor * v_angle_form_factor_faded * weight;
	Vector f2 =   -1.0 * ( j_xyz - i_xyz ).normalized();
	Vector f1 =    1.0 * cross( f2, j_xyz );
	// acceptor atom
	r1_atom_derivs[ i ].f1() -= dE_ddist * f1;
	r1_atom_derivs[ i ].f2() -= dE_ddist * f2;

	// Mg atom
	r2_atom_derivs[ j ].f1() += dE_ddist * f1;
	r2_atom_derivs[ j ].f2() += dE_ddist * f2;

	Real theta( 0.0 );
	//////////////////////////////////////////////////////////////////
	// Term 2: form factor for angle Mg -- Acceptor -- Acceptor-Base
	//////////////////////////////////////////////////////////////////
	// adapted from hbond_geom.cc
	if ( is_water ) {
		///////////////////////////////////////////
		Vector const OH1_xyz( rsd1.xyz( OH1 ) );
		Real acc_angle_form_factor_OH1_deriv = core::scoring::magnesium::get_gaussian_deriv( mg_potential_costheta_gaussian_parameter, cos_theta_OH1 );
		angle_p1_deriv(  j_xyz, i_xyz, OH1_xyz, theta, f1, f2);
		Real const dE_dcosAtheta_sin_theta_OH1 = 0.5 * dist_score * acc_angle_form_factor_OH1_deriv * v_angle_form_factor_faded * weight * sin( theta );

		// Mg atom
		r2_atom_derivs[ j ].f1() += dE_dcosAtheta_sin_theta_OH1 * f1;
		r2_atom_derivs[ j ].f2() += dE_dcosAtheta_sin_theta_OH1 * f2;

		// acceptor atom
		angle_p2_deriv(  OH1_xyz, i_xyz, j_xyz, theta, f1, f2);
		r1_atom_derivs[ i ].f1() += dE_dcosAtheta_sin_theta_OH1 * f1;
		r1_atom_derivs[ i ].f2() += dE_dcosAtheta_sin_theta_OH1 * f2;

		// acceptor base atom
		angle_p1_deriv(  OH1_xyz, i_xyz, j_xyz, theta, f1, f2);
		r1_atom_derivs[ OH1 ].f1() += dE_dcosAtheta_sin_theta_OH1 * f1;
		r1_atom_derivs[ OH1 ].f2() += dE_dcosAtheta_sin_theta_OH1 * f2;

		///////////////////////////////////////////
		Vector const OH2_xyz( rsd1.xyz( OH2 ) );
		Real acc_angle_form_factor_OH2_deriv = core::scoring::magnesium::get_gaussian_deriv( mg_potential_costheta_gaussian_parameter, cos_theta_OH2 );
		angle_p1_deriv(  j_xyz, i_xyz, OH2_xyz, theta, f1, f2);
		Real const dE_dcosAtheta_sin_theta_OH2 = 0.5 * dist_score * acc_angle_form_factor_OH2_deriv * v_angle_form_factor_faded * weight * sin( theta );

		// Mg atom
		r2_atom_derivs[ j ].f1() += dE_dcosAtheta_sin_theta_OH2 * f1;
		r2_atom_derivs[ j ].f2() += dE_dcosAtheta_sin_theta_OH2 * f2;

		// acceptor atom
		angle_p2_deriv(  OH2_xyz, i_xyz, j_xyz, theta, f1, f2);
		r1_atom_derivs[ i ].f1() += dE_dcosAtheta_sin_theta_OH2 * f1;
		r1_atom_derivs[ i ].f2() += dE_dcosAtheta_sin_theta_OH2 * f2;

		// acceptor base atom
		angle_p1_deriv(  OH2_xyz, i_xyz, j_xyz, theta, f1, f2);
		r1_atom_derivs[ OH2 ].f1() += dE_dcosAtheta_sin_theta_OH2 * f1;
		r1_atom_derivs[ OH2 ].f2() += dE_dcosAtheta_sin_theta_OH2 * f2;

	} else {
		Real acc_angle_form_factor_deriv = core::scoring::magnesium::get_gaussian_deriv( mg_potential_costheta_gaussian_parameter, cos_theta );
		angle_p1_deriv(  j_xyz, i_xyz, base_xyz, theta, f1, f2);
		Real const dE_dcosAtheta_sin_theta = dist_score * acc_angle_form_factor_deriv * v_angle_form_factor_faded * weight * sin( theta );

		// Mg atom
		r2_atom_derivs[ j ].f1() += dE_dcosAtheta_sin_theta * f1;
		r2_atom_derivs[ j ].f2() += dE_dcosAtheta_sin_theta * f2;

		// acceptor atom
		angle_p2_deriv(  base_xyz, i_xyz, j_xyz, theta, f1, f2);
		r1_atom_derivs[ i ].f1() += dE_dcosAtheta_sin_theta * f1;
		r1_atom_derivs[ i ].f2() += dE_dcosAtheta_sin_theta * f2;

		// acceptor base atom
		angle_p1_deriv(  base_xyz, i_xyz, j_xyz, theta, f1, f2);
		core::scoring::DerivVectorPair abase_deriv;
		abase_deriv.f1() = dE_dcosAtheta_sin_theta * f1;
		abase_deriv.f2() = dE_dcosAtheta_sin_theta * f2;

		runtime_assert( rsd1.heavyatom_is_an_acceptor( i ) );
		static core::scoring::hbonds::HBondOptions const hbond_options;
		chemical::Hybridization acc_hybrid( rsd1.atom_type( i ).hybridization() );
		assign_abase_derivs( hbond_options, rsd1, i, acc_hybrid, abase_deriv, 1.0, r1_atom_derivs );
	}

	//////////////////////////////////////////////////////////////////
	// Term 3: form factor for angle Acceptor -- Mg -- V
	//////////////////////////////////////////////////////////////////
	Real const v_angle_form_factor_deriv = ( 1.0 / ( 2.0 * v_angle_width2_ ) ) * exp( - ( 1.0 - cos_v_angle ) / (2.0 * v_angle_width2_ ) );
	Real const v_angle_form_factor_faded_deriv = ( 1.0 - v_angle_baseline ) * v_angle_form_factor_deriv;

	Size const v = core::scoring::magnesium::get_closest_orbital_axis( rsd2, i_xyz ) + 1; // offset is due to Mg(2+), then V1, V2, ...
	Vector const & v_xyz( rsd2.xyz( v ) );
	angle_p1_deriv( v_xyz, j_xyz, i_xyz, theta, f1, f2);
	// there's a -1.0 here because the angle in numeric::deriv::angle_p1_deriv is A-->M-->V, not A<--M-->V.
	Real const dE_dcosVtheta_sin_theta = -1.0 * dist_score * acc_angle_form_factor * v_angle_form_factor_faded_deriv * weight * sin( theta );
	r2_atom_derivs[ v ].f1() += dE_dcosVtheta_sin_theta * f1;
	r2_atom_derivs[ v ].f2() += dE_dcosVtheta_sin_theta * f2;

	// Mg atom
	angle_p2_deriv( i_xyz, j_xyz, v_xyz, theta, f1, f2);
	r2_atom_derivs[ j ].f1() += dE_dcosVtheta_sin_theta * f1;
	r2_atom_derivs[ j ].f2() += dE_dcosVtheta_sin_theta * f2;

	// acceptor atom
	angle_p1_deriv( i_xyz, j_xyz, v_xyz, theta, f1, f2);
	r1_atom_derivs[ i ].f1() += dE_dcosVtheta_sin_theta * f1;
	r1_atom_derivs[ i ].f2() += dE_dcosVtheta_sin_theta * f2;
}


///////////////////////////
void
MgEnergy::setup_for_minimizing_for_residue(
	conformation::Residue const &,
	pose::Pose const &,
	core::scoring::ScoreFunction const &,
	kinematics::MinimizerMapBase const &,
	basic::datacache::BasicDataCache &,
	core::scoring::ResSingleMinimizationData &
) const
{}

////////////////////////////////////////////////////////////////////////////////
// copied from scoring::geometric_solvation::GeometricSolEnergyEvaluator, which was itself copied from god knows where.
void
MgEnergy::setup_for_minimizing_for_residue_pair(
	conformation::Residue const & rsd1,
	conformation::Residue const & rsd2,
	pose::Pose const &,
	core::scoring::ScoreFunction const &,
	kinematics::MinimizerMapBase const &,
	core::scoring::ResSingleMinimizationData const &,
	core::scoring::ResSingleMinimizationData const &,
	core::scoring::ResPairMinimizationData & pair_data
) const
{
	using namespace core::scoring::etable::count_pair;
	CountPairFunctionCOP count_pair( utility::pointer::make_shared< CountPairAll >() );

	// update the existing nblist if it's already present in the min_data object
	core::scoring::ResiduePairNeighborListOP nblist( utility::pointer::static_pointer_cast< core::scoring::ResiduePairNeighborList > ( pair_data.get_data( core::scoring::mg_pair_nblist ) ));
	if ( ! nblist ) nblist = utility::pointer::make_shared< core::scoring::ResiduePairNeighborList >();

	/// STOLEN CODE!
	Real const tolerated_narrow_nblist_motion = 0.75; //option[ run::nblist_autoupdate_narrow ];
	Real const XX2 = std::pow( 5.2 + 2*tolerated_narrow_nblist_motion, 2 );

	nblist->initialize_from_residues( XX2, XX2, XX2, rsd1, rsd2, count_pair );

	pair_data.set_data( core::scoring::mg_pair_nblist, nblist );
}

/////////////
bool
MgEnergy::requires_a_setup_for_derivatives_for_residue_pair_opportunity( pose::Pose const & ) const
{
	return false;
}


/// @brief MgEnergy distance cutoff
Distance
MgEnergy::atomic_interaction_cutoff() const
{
	return 6.0;
}

/// @brief MgEnergy
void
MgEnergy::indicate_required_context_graphs( utility::vector1< bool > & /* context_graphs_required */ ) const
{
}

core::Size
MgEnergy::version() const
{
	return 1; // Initial versioning
}

} //scoring
} //core
