// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file  protocols/membrane/scoring/FaWaterToBilayerEnergy.hh
/// @brief Implicit Lipid Membrane Model Water-to-Bilayer transfer energy (one-body)
/// @author  Rebecca Alford (ralford3@jhu.edu)

#ifndef INCLUDED_protocols_membrane_scoring_FaWaterToBilayerEnergy_hh
#define INCLUDED_protocols_membrane_scoring_FaWaterToBilayerEnergy_hh

// Unit headers
#include <protocols/membrane/scoring/FaWaterToBilayerEnergy.fwd.hh>
#include <core/scoring/methods/ContextDependentOneBodyEnergy.hh>
#include <core/scoring/methods/EnergyMethodOptions.fwd.hh>

// Package headers
#include <protocols/membrane/scoring/MEnvAtomParams.fwd.hh>
#include <core/scoring/ScoreFunction.fwd.hh>


#include <core/pose/Pose.fwd.hh>

// Utility Headers
#include <utility/vector1.hh>

// C++ Headers

namespace protocols {
namespace membrane {
namespace scoring {

/// @brief Fullatom Membrane Environment Energy
class FaWaterToBilayerEnergy : public core::scoring::methods::ContextDependentOneBodyEnergy {

public:

	typedef core::scoring::methods::ContextDependentOneBodyEnergy parent;

	/// @brief Construct Energy Method from Etable
	FaWaterToBilayerEnergy( core::scoring::methods::EnergyMethodOptions const & options );

	/// @brief Clone Energy Method
	core::scoring::methods::EnergyMethodOP
	clone() const override;

	/// @brief Compute Per-Residue Energies
	void
	residue_energy(
		core::conformation::Residue const & rsd,
		core::pose::Pose const & pose,
		core::scoring::EnergyMap & emap
	) const override;

	/// @brief Fianlzie Total Per-Residue Energies
	void
	finalize_total_energy(
		core::pose::Pose & pose,
		core::scoring::ScoreFunction const &,
		core::scoring::EnergyMap & emap
	) const override;

	/// @brief Setup for Computing Derivatives
	void
	setup_for_derivatives(
		core::pose::Pose & pose,
		core::scoring::ScoreFunction const & scfxn
	) const override;

	/// @brief Evaluate Per-Atom Derivatives
	void
	eval_atom_derivative(
		core::id::AtomID const & id,
		core::pose::Pose const & pose,
		core::kinematics::DomainMap const & domain_map,
		core::scoring::ScoreFunction const & sfxn,
		core::scoring::EnergyMap const & emap,
		core::Vector & F1,
		core::Vector & F2
	) const override;

	/// @brief Fa_MbenvEnergy is context independent
	void
	indicate_required_context_graphs( utility::vector1< bool > & ) const override;

	/// @brief Setup Method for initial scoring
	void
	setup_for_scoring(
		core::pose::Pose & pose,
		core::scoring::ScoreFunction const &
	) const override;

	MEnvAtomParamsCOP
	get_menv_params_for_residue(
		core::pose::Pose const & pose,
		core::conformation::Residue const & rsd,
		core::Size atomno
	) const;

	/// @brief Evaluate Per-Atom Env term
	core::Real
	eval_fa_wtbe(
		MEnvAtomParams const & p
	) const;


private: // helper methods

	core::Size
	get_atype_index( std::string atype_name ) const;

	/// @brief Versioning
	core::Size version() const override;


private:

	// will clean this up later
	mutable utility::vector1< core::Real > memb_lk_dgrefce_;
	mutable utility::vector1< core::Real > water_lk_dgrefce_;
	mutable utility::vector1< std::string > atypes_list_;

	// Store mbenv weight when computing derivatives
	mutable core::Real fa_wtbe_weight_;
	bool use_fleming_de_;

};

} // scoring
} // membrane
} // protocols

#endif // INCLUDED_protocols_membrane_scoring_FaWaterToBilayerEnergy_hh
