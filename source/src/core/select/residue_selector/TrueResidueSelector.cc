// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington UW TechTransfer, email: license@u.washington.edu.

/// @file   core/select/residue_selector/TrueResidueSelector.cc
/// @brief  The TrueResidueSelector creates an appropriate all-true vector
/// @author Justin R. Porter

// Unit headers
#include <core/select/residue_selector/TrueResidueSelector.hh>
#include <core/select/residue_selector/ResidueSelectorCreators.hh>

// Package headers
#include <core/pose/Pose.fwd.hh>
#include <core/select/residue_selector/ResidueSelectorFactory.hh>

// Basic headers
#include <basic/datacache/DataMap.hh>

// Utility Headers
#include <utility/tag/Tag.hh>
#include <utility/string_util.hh>
#include <utility/vector1.hh>

// C++ headers
#include <utility/assert.hh>

namespace core {
namespace select {
namespace residue_selector {


TrueResidueSelector::TrueResidueSelector() :
	ResidueSelector()
{}

TrueResidueSelector::~TrueResidueSelector() {}

/// @brief Copy constructor
///
TrueResidueSelector::TrueResidueSelector( TrueResidueSelector const &) :
	ResidueSelector()
{}

/// @brief Clone operator.
/// @details Copy this object and return an owning pointer to the new object.
ResidueSelectorOP TrueResidueSelector::clone() const { return ResidueSelectorOP( new TrueResidueSelector(*this) ); }

ResidueSubset
TrueResidueSelector::apply( core::pose::Pose const & pose ) const
{
	return ResidueSubset( pose.total_residue(), true );
}

void TrueResidueSelector::parse_my_tag(
	utility::tag::TagCOP,
	basic::datacache::DataMap&
)
{}

std::string TrueResidueSelector::get_name() const {
	return TrueResidueSelector::class_name();
}

std::string TrueResidueSelector::class_name() {
	return "True";
}

ResidueSelectorOP
TrueResidueSelectorCreator::create_residue_selector() const {
	return ResidueSelectorOP( new TrueResidueSelector );
}

std::string
TrueResidueSelectorCreator::keyname() const {
	return TrueResidueSelector::class_name();
}

} //namespace residue_selector
} //namespace select
} //namespace core
