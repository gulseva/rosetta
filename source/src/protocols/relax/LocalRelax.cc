// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file src/protocols/relax/LocalRelax.cc
/// @brief A relax protocol that iteratively cart relaxes clustered subsets of residues
/// @author Frank DiMaio


#include <protocols/relax/LocalRelax.hh>
#include <protocols/relax/LocalRelaxCreator.hh>

#include <core/chemical/ChemicalManager.fwd.hh>
#include <core/conformation/Conformation.hh>
#include <core/conformation/Residue.hh>
#include <core/conformation/symmetry/SymmetricConformation.hh>
#include <core/conformation/symmetry/SymmetryInfo.hh>
#include <core/optimization/CartesianMinimizer.hh>
#include <core/optimization/MinimizerOptions.hh>
#include <core/pack/pack_rotamers.hh>
#include <core/pack/task/operation/TaskOperations.hh>
#include <core/pack/task/PackerTask.hh>
#include <core/pack/task/TaskFactory.hh>
#include <core/pose/Pose.hh>
#include <core/pose/symmetry/util.hh>
#include <core/scoring/ScoreFunction.hh>
#include <core/scoring/ScoreFunctionFactory.hh>
#include <core/scoring/electron_density/util.hh>
#include <core/scoring/constraints/util.hh>
#include <core/types.hh>
#include <core/util/SwitchResidueTypeSet.hh>


#include <utility/vector1.hh>
#include <utility/string_util.hh>

#include <numeric/xyzVector.hh>



#include <basic/Tracer.hh>
#include <basic/options/option.hh>
#include <basic/datacache/DataMap.hh>

#define foreach_ for

// option includes
#include <basic/options/keys/relax.OptionKeys.gen.hh>
#include <basic/options/keys/edensity.OptionKeys.gen.hh>
#include <basic/options/keys/constraints.OptionKeys.gen.hh>

#include <fstream>

#include <string>
// XSD XRW Includes
#include <utility/tag/XMLSchemaGeneration.hh>
#include <protocols/moves/mover_schemas.hh>

#include <core/kinematics/MoveMap.hh> // AUTO IWYU For MoveMap
#include <core/pack/task/ResidueLevelTask_.hh> // AUTO IWYU For ResidueLevelTask_
#include <utility/tag/Tag.hh> // AUTO IWYU For Tag


namespace protocols {
namespace relax {


using namespace core;

static basic::Tracer TR("LocalRelax");






LocalRelax::LocalRelax() {
	using namespace basic::options;
	using namespace basic::options::OptionKeys;

	NCYC_ = option[ basic::options::OptionKeys::relax::default_repeats ](); // n relax cycles
	NEXP_ = 2; // n expansions
	K_ = 0; // CB dist cut
	max_iter_ = 200;
	verbose_ = false;
	ramp_cart_ = false;

	ramp_schedule_.push_back(0.02);
	ramp_schedule_.push_back(0.25);
	ramp_schedule_.push_back(0.55);
	ramp_schedule_.push_back(1.0);

	pack_sfxn_ = core::scoring::get_score_function();
	min_sfxn_ = core::scoring::get_score_function();

	if ( option[ edensity::mapfile ].user() ) {
		core::scoring::electron_density::add_dens_scores_from_cmdline_to_scorefxn( *pack_sfxn_ );
		core::scoring::electron_density::add_dens_scores_from_cmdline_to_scorefxn( *min_sfxn_ );
	}

	if ( option[ OptionKeys::constraints::cst_fa_weight].user() ) {
		core::scoring::constraints::add_fa_constraints_from_cmdline_to_scorefxn( *pack_sfxn_ );
		core::scoring::constraints::add_fa_constraints_from_cmdline_to_scorefxn( *min_sfxn_ );
	} else {
		core::scoring::constraints::add_constraints_from_cmdline_to_scorefxn( *pack_sfxn_ );
		core::scoring::constraints::add_constraints_from_cmdline_to_scorefxn( *min_sfxn_ );
	}
}


void
LocalRelax::optimization_loop(
	Pose & pose,
	core::pack::task::PackerTaskOP ptask,
	core::kinematics::MoveMapOP mm,
	core::Real fa_rep_scale,
	core::Real min_tol)
{
	using namespace core::scoring;

	// minpack+cartmin
	core::optimization::CartesianMinimizer minimizer;

	core::scoring::ScoreFunctionOP local_pack_sf = pack_sfxn_->clone();
	core::scoring::ScoreFunctionOP local_min_sf = min_sfxn_->clone();

	local_pack_sf->set_weight( fa_rep, fa_rep_scale*pack_sfxn_->get_weight( fa_rep ) );
	local_min_sf->set_weight( fa_rep, fa_rep_scale*min_sfxn_->get_weight( fa_rep ) );

	if ( ramp_cart_ ) {
		core::Real cart_scale = std::max( fa_rep_scale, 0.1 );
		local_pack_sf->set_weight( cart_bonded, cart_scale*pack_sfxn_->get_weight( cart_bonded ) );
		local_min_sf->set_weight( cart_bonded, cart_scale*min_sfxn_->get_weight( cart_bonded ) );
		local_pack_sf->set_weight( cart_bonded_angle, cart_scale*pack_sfxn_->get_weight( cart_bonded_angle ) );
		local_min_sf->set_weight( cart_bonded_angle, cart_scale*min_sfxn_->get_weight( cart_bonded_angle ) );
		local_pack_sf->set_weight( cart_bonded_length, cart_scale*pack_sfxn_->get_weight( cart_bonded_length ) );
		local_min_sf->set_weight( cart_bonded_length, cart_scale*min_sfxn_->get_weight( cart_bonded_length ) );
		local_pack_sf->set_weight( cart_bonded_torsion, cart_scale*pack_sfxn_->get_weight( cart_bonded_torsion ) );
		local_min_sf->set_weight( cart_bonded_torsion, cart_scale*min_sfxn_->get_weight( cart_bonded_torsion ) );

	}

	core::optimization::MinimizerOptions options( "lbfgs_armijo_nonmonotone", min_tol, true, false, false );
	options.max_iter(max_iter_);
	core::pack::pack_rotamers( pose, *local_pack_sf, ptask );
	minimizer.run( pose, *mm, *local_min_sf, options );

	// AMW: cppcheck flags this but unnecessarily so
	static int dump_idx=1;
	if ( verbose_ ) {
		std::string name = "opt_"+utility::to_string( dump_idx++ )+".pdb";
		TR << "Write " << name << std::endl;
		pose.dump_pdb( name );
	}
}

utility::vector1< utility::vector1<bool> >
LocalRelax::get_neighbor_graph(Pose const & pose) {
	using namespace core;
	using namespace core::scoring;
	core::Size const nres = pose.size();

	// grab symminfo (if defined) from the pose
	core::conformation::symmetry::SymmetryInfoCOP symminfo=nullptr;
	if ( core::pose::symmetry::is_symmetric(pose) ) {
		symminfo = dynamic_cast<const core::conformation::symmetry::SymmetricConformation &>(pose.conformation()).Symmetry_Info();
	}
	utility::vector1< utility::vector1<bool> >neighbor;
	neighbor.resize( nres );

	// core::Real K=K_;

	for ( core::Size i=1, i_end = nres; i<= i_end; ++i ) {
		if ( symminfo && !symminfo->bb_is_independent( i ) ) continue;
		neighbor[i].resize(nres, false);
		neighbor[i][i] = true;


		conformation::Residue const & rsd1( pose.residue( i ) );
		for ( core::Size j=1, j_end = nres; j<= j_end; ++j ) {
			conformation::Residue const & rsd2( pose.residue( j ) );
			if ( i==j ) continue;
			numeric::xyzVector<core::Real> const & nbr_atom_xyz_i(rsd1.atom(rsd1.nbr_atom()).xyz());
			core::Real const & nbr_atom_radius_i(rsd1.nbr_radius());

			numeric::xyzVector<core::Real> const & nbr_atom_xyz_j(rsd2.atom(rsd2.nbr_atom()).xyz());
			core::Real const & nbr_atom_radius_j(rsd2.nbr_radius());

			core::Real const dist(nbr_atom_xyz_i.distance(nbr_atom_xyz_j));
			core::Real const interact_threshold(nbr_atom_radius_i + nbr_atom_radius_j+K_);

			if ( dist <= interact_threshold ) {
				core::Size j_asu(j);
				if ( symminfo && !symminfo->bb_is_independent( j ) ) {
					j_asu = symminfo->bb_follows( j );
				}
				neighbor[i][j_asu] = true;
			}
		}
	}
	return neighbor;
}


void
LocalRelax::parse_my_tag(
	utility::tag::TagCOP tag,
	basic::datacache::DataMap & data
) {
	using namespace basic::options;
	using namespace core::scoring;

	// scorefxns
	if ( tag->hasOption( "scorefxn" ) ) {
		std::string const scorefxn_name( tag->getOption<std::string>( "scorefxn" ) );
		pack_sfxn_ = (data.get_ptr< ScoreFunction >( "scorefxns", scorefxn_name ));
		min_sfxn_ = (data.get_ptr< ScoreFunction >( "scorefxns", scorefxn_name ));
	}
	if ( tag->hasOption( "pack_scorefxn" ) ) {
		std::string const scorefxn_name( tag->getOption<std::string>( "pack_scorefxn" ) );
		pack_sfxn_ = (data.get_ptr< ScoreFunction >( "scorefxns", scorefxn_name ));
	}
	if ( tag->hasOption( "min_scorefxn" ) ) {
		std::string const scorefxn_name( tag->getOption<std::string>( "min_scorefxn" ) );
		min_sfxn_ = (data.get_ptr< ScoreFunction >( "scorefxns", scorefxn_name ));
	}

	if ( tag->hasOption( "ncyc" ) ) {
		NCYC_ = tag->getOption< int >( "ncyc" );
	}
	if ( tag->hasOption( "nexp" ) ) {
		NEXP_ = tag->getOption< int >( "nexp" );
	}
	if ( tag->hasOption( "K" ) ) {
		K_ = tag->getOption< int >( "K" );
	}

	if ( tag->hasOption( "max_iter" ) ) {
		max_iter_ = tag->getOption< int >( "max_iter" );
	}


	if ( tag->hasOption( "ramp_schedule" ) ) {
		std::string ramp_schedule_str = tag->getOption< std::string >( "ramp_schedule" );
		utility::vector1< std::string > ramp_schedule_strs = utility::string_split ( ramp_schedule_str, ',' );
		ramp_schedule_.clear();
		for ( core::Size i=1; i<= ramp_schedule_strs.size(); ++i ) {
			ramp_schedule_.push_back( atof(ramp_schedule_strs[i].c_str()) );
		}
		runtime_assert( ramp_schedule_.size() >= 1);
	}

	verbose_ = tag->getOption< bool >( "verbose" , false );
	ramp_cart_ = tag->getOption< bool >( "ramp_cart" , false );
}


void
LocalRelax::apply( core::pose::Pose & pose) {
	if ( !pose.is_fullatom() ) {
		core::util::switch_to_residue_type_set( pose, core::chemical::FULL_ATOM_t );
	}

	core::Size const nres = pose.size();
	core::Size nres_asu = nres;

	// set up symm
	core::conformation::symmetry::SymmetryInfoCOP symminfo=nullptr;
	if ( core::pose::symmetry::is_symmetric(pose) )  {
		symminfo = dynamic_cast<const core::conformation::symmetry::SymmetricConformation &>(pose.conformation()).Symmetry_Info();
		nres_asu = symminfo->num_independent_residues();
	}

	// set up packer task [to do: make this RS selectible]
	core::pack::task::TaskFactoryOP task( utility::pointer::make_shared< core::pack::task::TaskFactory >() );
	task->push_back( utility::pointer::make_shared< core::pack::task::operation::InitializeFromCommandline >() );
	task->push_back( utility::pointer::make_shared< core::pack::task::operation::RestrictToRepacking >() );
	task->push_back( utility::pointer::make_shared< core::pack::task::operation::IncludeCurrent >() );
	core::pack::task::PackerTaskOP ptask_resfile = task->create_task_and_apply_taskoperations( pose );

	// for each residue
	for ( core::Size cyc = 1; cyc <= NCYC_; ++cyc ) {
		for ( core::Size innercyc = 1; innercyc <= ramp_schedule_.size(); ++innercyc ) {
			utility::vector1< utility::vector1<bool> > const neighbor(get_neighbor_graph( pose ));

			// "priority list" on residues
			//   - sort by connectedness
			utility::vector1< core::Size > neighborcounts(nres, 0);
			for ( core::Size i=1, i_end = nres; i<= i_end; ++i ) {
				if ( !symminfo || symminfo->bb_is_independent(i) ) {
					for ( core::Size j=1; j<=nres; ++j ) if ( neighbor[i][j] ) neighborcounts[j]++;
				}
			}

			utility::vector1<bool> shell0, shell1, visited(nres, false);

			// mark non-packable as visited
			for ( core::Size i=1; i<=nres; ++i ) {
				if ( !ptask_resfile->pack_residue( i ) ) visited[i] = true;
				if ( symminfo && !symminfo->bb_is_independent(i) ) visited[i] = true;
			}

			core::Size nvis=0;
			// main loop
			while ( true ) {
				// find most connected residue
				core::Size maxneighb=0;
				core::Size currres=0;
				for ( core::Size i=1; i<=nres; ++i ) {
					if ( !visited[i] && neighborcounts[i] > maxneighb ) {
						maxneighb = neighborcounts[i];
						currres = i;
					}
				}

				if ( maxneighb==0 ) {
					// all done
					break;
				} else if ( maxneighb<10 || (nres_asu-nvis)<25 ) {
					TR << "PACK SURFACE" << std::endl;

					utility::vector1<bool> neigh_merge(nres, false);
					for ( core::Size i=1; i<=nres; ++i ) {
						if ( visited[i] ) continue;
						if ( symminfo && !symminfo->bb_is_independent(i) ) continue;
						for ( core::Size j=1; j<=nres; ++j ) {
							neigh_merge[j] = neigh_merge[j] || neighbor[i][j];
						}
					}

					// "surface pack" << generally lots of surface residues in small clusters.  Pack them all at once
					shell0 = neigh_merge;
					shell1 = shell0;
					for ( core::Size j=1; j<=nres; ++j ) {
						if ( shell0[j] ) {
							for ( core::Size k=1; k<=nres; ++k ) {
								if ( !shell0[k] && neigh_merge[k] ) shell1[k] = true;
							}
						}
					}
				} else {
					shell1 = neighbor[currres];
					for ( core::Size i=1; i<=NEXP_; ++i ) {
						shell0 = shell1;
						for ( core::Size j=1; j<=nres; ++j ) {
							if ( shell0[j] ) {
								for ( core::Size k=1; k<=nres; ++k ) {
									if ( !shell0[k] && neighbor[j][k] ) shell1[k] = true;
								}
							}
						}
					}
				}

				// build 1 residue packer task
				core::pack::task::PackerTaskOP ptask_working(core::pack::task::TaskFactory::create_packer_task( pose ));
				ptask_working->restrict_to_residues(shell1);
				ptask_working->or_include_current(true);

				for ( core::Size j=1, j_end = nres; j<= j_end; ++j ) {
					if ( shell0[j] ) {
						visited[j] = true;
						dynamic_cast<core::pack::task::ResidueLevelTask_&>
							(ptask_working->nonconst_residue_task(j)).update_commutative( ptask_resfile->nonconst_residue_task(j) );
					} else if ( shell1[j] ) {
						ptask_working->nonconst_residue_task(j).restrict_to_repacking();
						ptask_working->nonconst_residue_task(j).or_include_current(true);
					}
				}

				// set up movemap
				core::kinematics::MoveMapOP mm( utility::pointer::make_shared< core::kinematics::MoveMap >() );
				mm->set_jump(true);
				for ( core::Size j=1, j_end = nres; j<= j_end; ++j ) {
					if ( shell1[j] ) {
						mm->set_bb(j, false); mm->set_chi(j, true);
					} else {
						mm->set_bb(j, false); mm->set_chi(j, false);
					}
				}

				for ( core::Size j=1, j_end = nres; j<= j_end; ++j ) {
					if ( shell0[j] ) {
						// allow a window of bb movement around each central residue
						mm->set_bb(j, true);
						mm->set_chi(j, true);
						if ( j<nres ) { mm->set_bb(j+1, true); mm->set_chi(j+1, true); }
						if ( j>1 )    { mm->set_bb(j-1, true); mm->set_chi(j-1, true); }
					}
				}

				if ( core::pose::symmetry::is_symmetric(pose) )  {
					core::pose::symmetry::make_symmetric_movemap( pose, *mm );
				}

				nvis = 0;
				for ( core::Size j=1, j_end = nres; j<= j_end; ++j ) {
					if ( symminfo && !symminfo->bb_is_independent(j) ) continue;
					if ( visited[j] ) nvis++;
				}

				// optimize
				optimization_loop( pose, ptask_working, mm,  ramp_schedule_[innercyc], 1e-4 );
				TR << "[" << cyc << "." << innercyc << "] res " << currres << " [ nneigh=" << maxneighb << " ] ("
					<< nvis << "/" << nres_asu << ")  E=" << (*min_sfxn_)(pose)
					<< "  ramp=" << ramp_schedule_[innercyc] << std::endl;
			}
		}
	}
}

std::string LocalRelax::get_name() const {
	return mover_name();
}

std::string LocalRelax::mover_name() {
	return "LocalRelax";
}

void LocalRelax::provide_xml_schema( utility::tag::XMLSchemaDefinition & xsd )
{
	// TO DO!
	using namespace utility::tag;
	AttributeList attlist; // TO DO: add attributes to this list
	attlist + XMLSchemaAttribute( "scorefxn", xs_string, "Sets the scorefxn for both pack and min stages.")
		+ XMLSchemaAttribute( "pack_scorefxn", xs_string, "Sets the scorefxn for both packing only.")
		+ XMLSchemaAttribute( "min_scorefxn", xs_string, "Sets the scorefxn for min only")
		+ XMLSchemaAttribute( "ncyc", xs_integer, "Number of cycles to perform localrelax")
		+ XMLSchemaAttribute( "nexp", xs_integer, "Number of expansions to perform")
		+ XMLSchemaAttribute( "K", xs_integer, "K is added to NBR_RADIUS-i and NBR_RADIUS-j to determine the size of the packing/minimization shells")
		+ XMLSchemaAttribute( "max_iter", xs_integer, "maximum iterations to perform in minimization")
		+ XMLSchemaAttribute( "ramp_schedule", xs_string, "XRW TO DO")
		+ XMLSchemaAttribute::attribute_w_default( "verbose", xsct_rosetta_bool, "not really verbose, just dump intermediate files to the local directory", "false")
		+ XMLSchemaAttribute::attribute_w_default( "ramp_cart", xsct_rosetta_bool, "XRW TO DO", "false");

	protocols::moves::xsd_type_definition_w_attributes( xsd, mover_name(), "XRW TO DO", attlist );
}

std::string LocalRelaxCreator::keyname() const {
	return LocalRelax::mover_name();
}

protocols::moves::MoverOP
LocalRelaxCreator::create_mover() const {
	return utility::pointer::make_shared< LocalRelax >();
}

void LocalRelaxCreator::provide_xml_schema( utility::tag::XMLSchemaDefinition & xsd ) const
{
	LocalRelax::provide_xml_schema( xsd );
}


}
}

