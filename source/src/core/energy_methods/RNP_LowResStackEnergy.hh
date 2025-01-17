// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file   core/scoring/rna/RNP_LowResStackEnergy.hh
/// @brief  Statistically derived rotamer pair potential class declaration
/// @author Rhiju Das


#ifndef INCLUDED_core_scoring_rna_RNP_LowResStackEnergy_hh
#define INCLUDED_core_scoring_rna_RNP_LowResStackEnergy_hh

// Unit Headers
#include <core/energy_methods/RNP_LowResStackEnergy.fwd.hh>
#include <core/scoring/rna/RNP_LowResStackData.fwd.hh>

// Package headers
#include <core/scoring/methods/ContextIndependentTwoBodyEnergy.hh>

// Project headers
#include <core/pose/Pose.fwd.hh>

#include <utility/vector1.hh>


// Utility headers


namespace core {
namespace energy_methods {


class RNP_LowResStackEnergy : public core::scoring::methods::ContextIndependentTwoBodyEnergy  {
public:
	typedef core::scoring::methods::ContextIndependentTwoBodyEnergy  parent;
public:


	RNP_LowResStackEnergy();


	/// clone
	core::scoring::methods::EnergyMethodOP
	clone() const override;

	/////////////////////////////////////////////////////////////////////////////
	// scoring
	/////////////////////////////////////////////////////////////////////////////

	//virtual
	//void
	//setup_for_scoring( pose::Pose & pose, core::scoring::ScoreFunction const & ) const;

	//virtual
	//void
	//setup_for_derivatives( pose::Pose & pose, core::scoring::ScoreFunction const & ) const;

	//virtual
	//void
	//setup_for_packing( pose::Pose & pose, utility::vector1< bool > const &, utility::vector1< bool > const & designing_residues ) const {};

	void
	residue_pair_energy(
		conformation::Residue const & rsd1,
		conformation::Residue const & rsd2,
		pose::Pose const &,
		core::scoring::ScoreFunction const &,
		core::scoring::EnergyMap & emap
	) const override;

	void
	eval_intrares_energy(
		conformation::Residue const &,
		pose::Pose const &,
		core::scoring::ScoreFunction const &,
		core::scoring::EnergyMap &
	) const override {}

	// virtual
	// void
	// eval_atom_derivative(
	//  id::AtomID const & atom_id,
	//  pose::Pose const & pose,
	//  kinematics::DomainMap const & domain_map,
	//  core::scoring::ScoreFunction const & scorefxn,
	//  core::scoring::EnergyMap const & weights,
	//  Vector & F1,
	//  Vector & F2
	// ) const {};

	bool
	defines_intrares_energy( core::scoring::EnergyMap const & /*weights*/ ) const override { return false; }

	// virtual
	// void
	// finalize_total_energy(
	//  pose::Pose & pose,
	//  core::scoring::ScoreFunction const &,
	//  core::scoring::EnergyMap &// totals
	// ) const;

	Distance
	atomic_interaction_cutoff() const override;

	void indicate_required_context_graphs( utility::vector1< bool > & ) const override {}


	/////////////////////////////////////////////////////////////////////////////
	// data
	/////////////////////////////////////////////////////////////////////////////

private:

	// const-ref to scoring database
	core::scoring::rna::RNP_LowResStackData const & potential_;

	core::Size version() const override;

};


} //scoring
} //core

#endif // INCLUDED_core_scoring_ScoreFunction_HH
