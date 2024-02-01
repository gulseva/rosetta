// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file bootcamp/BootCampMover.cc
/// @brief A test mover
/// @author gulseva (alican@gulsevinlab.org)

// Unit headers
#include <bootcamp/BootCampMover.hh>
#include <bootcamp/BootCampMoverCreator.hh>

// Core headers
#include <core/pose/Pose.hh>

// Basic/Utility headers
#include <basic/Tracer.hh>
#include <utility/tag/Tag.hh>
#include <utility/pointer/memory.hh>

// XSD Includes
#include <utility/tag/XMLSchemaGeneration.hh>
#include <protocols/moves/mover_schemas.hh>

// Citation Manager
#include <utility/vector1.hh>
#include <basic/citation_manager/UnpublishedModuleInfo.hh>


//Other headers
#include <iostream>
#include </data/programs/Rosetta/rosetta/source/src/basic/options/option.hh>
#include </data/programs/Rosetta/rosetta/source/src/devel/init.hh>
#include <basic/options/keys/in.OptionKeys.gen.hh>
#include <core/import_pose/import_pose.hh>
#include <utility/pointer/owning_ptr.hh>
#include <core/pose/Pose.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/scoring/ScoreFunction.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/scoring/ScoreFunctionFactory.hh>
#include <numeric/random/random.hh>
#include <basic/Tracer.hh>
#include <protocols/moves/MonteCarlo.hh>
#include <core/pack/task/PackerTask.hh>
#include <core/pack/task/TaskFactory.hh>
#include <core/pose/Pose.hh>
#include <core/pack/pack_rotamers.hh>
#include <core/kinematics/MoveMap.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/optimization/MinimizerOptions.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/optimization/AtomTreeMinimizer.hh>


static basic::Tracer TR( "bootcamp.BootCampMover" );

namespace bootcamp {

	/////////////////////
	/// Constructors  ///
	/////////////////////

/// @brief Default constructor
BootCampMover::BootCampMover():
	protocols::moves::Mover( BootCampMover::mover_name() )
{

}

////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor (important for properly forward-declaring smart-pointer members)
BootCampMover::~BootCampMover(){}

////////////////////////////////////////////////////////////////////////////////
	/// Mover Methods ///
	/////////////////////

/// @brief Apply the mover
void
BootCampMover::apply( core::pose::Pose& ){

	utility::vector1< std::string > filenames = basic::options::option[ basic::options::OptionKeys::in::file::s ].value();

        //Define the pose and the default score function objects
        core::pose::PoseOP mypose = core::import_pose::pose_from_file( filenames[1] );
        core::scoring::ScoreFunctionOP sfxn = core::scoring::get_score_function();

        //Calculate and print the score of the given pose
        core::Real score = sfxn->score( *mypose );
        std::cout << score << std::endl;

        //Create a Monte Carlo constructor mc that takes as input a pose and a score function, plus a temperature
        protocols::moves::MonteCarlo mc(*mypose, *sfxn, 0.8);

        //Create a copy pose object
        core::pose::Pose copy_pose;

        for (core::Size i = 0; i < 10; i++) {

        //Define a random generator object and calculate a random number
        numeric::random::RandomGenerator &generator = numeric::random::rg();
        core::Real uniform_random_number = generator.uniform();
        core::Real pert1 = generator.uniform();
        core::Real pert2 = generator.uniform();
        //Calculate the total number of residues in the given pose
        core::Size N = mypose->total_residue();

        //Define a random position based on the random number generator
        core::Size position = static_cast< core::Size > ( uniform_random_number * N + 1 );

        //Calculate the phi and psi values of the random position
        core::Real phi_value = mypose->phi(position);
        core::Real psi_value = mypose->psi(position);

        //Set new phi/psi values and apply the boltzmann method
        mypose->set_phi(position, phi_value + pert1);
        mypose->set_psi(position, psi_value + pert2);
        mc.boltzmann(*mypose);

        //Define a packer task, restrict to repacking and repack the pose with the given scorefunction
        core::pack::task::PackerTaskOP repack_task = core::pack::task::TaskFactory::create_packer_task( *mypose );
        repack_task->restrict_to_repacking();
        core::pack::pack_rotamers( *mypose, *sfxn, repack_task );

        //Create a movemap to fix or move parts of a residue
        core::kinematics::MoveMap mm;
        mm.set_bb( true );
        mm.set_chi( true );
        core::optimization::MinimizerOptions min_opts( "lbfgs_armijo_atol", 0.01, true );
        core::optimization::AtomTreeMinimizer atm;

        copy_pose = *mypose;
        atm.run( copy_pose, mm, *sfxn, min_opts );
        *mypose = copy_pose;

        //Print the score of the pose and the corresponding phi/psi values
        core::Real score = sfxn->score( *mypose );
        TR << "The score for the step " << i << "is: " << score << std::endl;
        TR << "Phi: " << phi_value << " Psi: " << psi_value << std::endl;
	}
}
////////////////////////////////////////////////////////////////////////////////
/// @brief Show the contents of the Mover
void
BootCampMover::show(std::ostream & output) const
{
	protocols::moves::Mover::show(output);
}

////////////////////////////////////////////////////////////////////////////////
	/// Rosetta Scripts Support ///
	///////////////////////////////

/// @brief parse XML tag (to use this Mover in Rosetta Scripts)
void
BootCampMover::parse_my_tag(
	utility::tag::TagCOP ,
	basic::datacache::DataMap&
) {

}
void BootCampMover::provide_xml_schema( utility::tag::XMLSchemaDefinition & xsd )
{

	using namespace utility::tag;
	AttributeList attlist;

	//here you should write code to describe the XML Schema for the class.  If it has only attributes, simply fill the probided AttributeList.

	protocols::moves::xsd_type_definition_w_attributes( xsd, mover_name(), "A test mover", attlist );
}


////////////////////////////////////////////////////////////////////////////////
/// @brief required in the context of the parser/scripting scheme
protocols::moves::MoverOP
BootCampMover::fresh_instance() const
{
	return utility::pointer::make_shared< BootCampMover >();
}

/// @brief required in the context of the parser/scripting scheme
protocols::moves::MoverOP
BootCampMover::clone() const
{
	return utility::pointer::make_shared< BootCampMover >( *this );
}

std::string BootCampMover::get_name() const {
	return mover_name();
}

std::string BootCampMover::mover_name() {
	return "BootCampMover";
}



/////////////// Creator ///////////////

protocols::moves::MoverOP
BootCampMoverCreator::create_mover() const
{
	return utility::pointer::make_shared< BootCampMover >();
}

std::string
BootCampMoverCreator::keyname() const
{
	return BootCampMover::mover_name();
}

void BootCampMoverCreator::provide_xml_schema( utility::tag::XMLSchemaDefinition & xsd ) const
{
	BootCampMover::provide_xml_schema( xsd );
}

/// @brief This mover is unpublished.  It returns gulseva as its author.
void
BootCampMover::provide_citation_info(basic::citation_manager::CitationCollectionList & citations ) const {
	citations.add(
		utility::pointer::make_shared< basic::citation_manager::UnpublishedModuleInfo >(
		"BootCampMover", basic::citation_manager::CitedModuleType::Mover,
		"gulseva",
		"TODO: institution",
		"alican@gulsevinlab.org",
		"Wrote the BootCampMover."
		)
	);
}


////////////////////////////////////////////////////////////////////////////////
	/// private methods ///
	///////////////////////


std::ostream &
operator<<( std::ostream & os, BootCampMover const & mover )
{
	mover.show(os);
	return os;
}

} //bootcamp
