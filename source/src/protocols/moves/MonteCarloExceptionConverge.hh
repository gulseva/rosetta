// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file
/// @brief
/// @author


#ifndef INCLUDED_protocols_moves_MonteCarloExceptionConverge_hh
#define INCLUDED_protocols_moves_MonteCarloExceptionConverge_hh


// type headers

// unit headers
#include <protocols/moves/MonteCarloExceptionConverge.fwd.hh>
#include <protocols/moves/MonteCarlo.fwd.hh>
#include <utility/excn/Exceptions.hh>

// package headers
#include <core/pose/Pose.fwd.hh>

// utility headers
#include <utility/VirtualBase.hh>
// #include "utility/sys_util.h"

// C++ headers
#include <string>



// Forward declarations

namespace protocols {
namespace moves {

class EXCN_Converged : public utility::excn::Exception {
public:
	EXCN_Converged(char const *file, int line, std::string const & m) : Exception(file, line, m + "\nexit protocol because structure is converged" ) {};
};

class MonteCarloExceptionConverge : public utility::VirtualBase {
public:
	virtual bool operator() ( const core::pose::Pose & pose, MonteCarlo const&, bool reject ) = 0; //throw exception EXCN_Converged if positive
protected:
};

} // moves
} // rosetta

#endif
