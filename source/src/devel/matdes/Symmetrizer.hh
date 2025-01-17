// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.
//
/// @file
/// @brief
/// @author Neil King ( neilking@uw.edu )
/// @author Javier Castellanos ( javiercv@uw.edu )

#ifndef INCLUDED_devel_matdes_Symmetrizer_HH
#define INCLUDED_devel_matdes_Symmetrizer_HH

// Unit headers
#include <devel/matdes/Symmetrizer.fwd.hh>

// Package headers

// project headers
#include <protocols/moves/Mover.hh>
#include <core/pose/Pose.fwd.hh>
#include <core/scoring/ScoreFunction.fwd.hh>
#include <basic/datacache/DataMap.fwd.hh>
#include <utility/tag/Tag.fwd.hh>

namespace devel {
namespace matdes {

class Symmetrizer : public protocols::moves::Mover {
	typedef core::pose::Pose Pose;
	typedef core::Real Real;
	typedef core::Size Size;
	typedef core::scoring::ScoreFunction ScoreFunction;
	typedef core::scoring::ScoreFunctionOP ScoreFunctionOP;
	typedef core::scoring::ScoreFunctionCOP ScoreFunctionCOP;
	typedef protocols::moves::MoverOP MoverOP;
	typedef basic::datacache::DataMap DataMap;
	typedef utility::tag::TagCOP TagCOP;

public:
	Symmetrizer();

	// --- virtual functions from mover ---
	void apply(Pose& pose) override;

	// --- virtual copy constructors
	MoverOP clone() const override;


	/// @brief create this type of object
	MoverOP fresh_instance() const override;


	void parse_my_tag( TagCOP tag,
		basic::datacache::DataMap & data
	) override;

	std::string
	get_name() const override;

	static
	std::string
	mover_name();

	static
	void
	provide_xml_schema( utility::tag::XMLSchemaDefinition & xsd );


private:
	Real get_radial_disp();
	Real get_angle();

private:
	std::string symm_file_;
	Real radial_disp_, radial_disp_min_, radial_disp_max_;
	Real angle_, angle_min_, angle_max_;
	Real radial_disp_delta_, angle_delta_;
	char symmetry_axis_;

	bool explore_grid_;
	std::string sampling_mode_;
};

} // matdes
} // devel
#endif
