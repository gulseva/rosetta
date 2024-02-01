// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file bootcamp/BootCampMover.fwd.hh
/// @brief A test mover
/// @author gulseva (alican@gulsevinlab.org)

#ifndef INCLUDED_bootcamp_BootCampMover_fwd_hh
#define INCLUDED_bootcamp_BootCampMover_fwd_hh

// Utility headers
#include <utility/pointer/owning_ptr.hh>


// Forward
namespace bootcamp {

class BootCampMover;

using BootCampMoverOP = utility::pointer::shared_ptr< BootCampMover >;
using BootCampMoverCOP = utility::pointer::shared_ptr< BootCampMover const >;

} //bootcamp

#endif //INCLUDED_bootcamp_BootCampMover_fwd_hh
