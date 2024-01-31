// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// // vi: set ts=2 noet:
// //
// // (c) Copyright Rosetta Commons Member Institutions.
// // (c) This file is part of the Rosetta software suite and is made available under license.
// // (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// // (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// // (c) addressed to University of Washington UW TechTransfer, email: license@u.washington.edu.
//
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
#include <core/kinematics/FoldTree.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/optimization/MinimizerOptions.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/optimization/AtomTreeMinimizer.hh>
#include </data/programs/Rosetta/rosetta/source/src/protocols/moves/MoverStatistics.hh>
#include </data/programs/Rosetta/rosetta/source/src/protocols/moves/DsspMover.hh>
#include </data/programs/Rosetta/rosetta/source/src/core/scoring/dssp/Dssp.hh>

static basic::Tracer TR( "core.io.pdb.file_data" );


//Function 2
core::kinematics::FoldTree fold_tree_from_dssp_string(std::string &word){
        core::kinematics::FoldTree ftree = core::kinematics::FoldTree();
        return ftree;
}

//Function 1 
core::kinematics::FoldTree fold_tree_from_ss(core::pose::Pose & pose){
	core::scoring::dssp::Dssp dsspmover(pose);
	std::string output_sequence = dsspmover.get_dssp_secstruct();
	core::kinematics::FoldTree ftree = fold_tree_from_dssp_string(output_sequence);
	std::cout << "The SS pattern is: " << output_sequence << std::endl;
	std::cout << "The fold tree is: " << ftree << std::endl;
	return ftree;
}

//Function 2 
//core::kinematics::FoldTree fold_tree_from_ss(std::string input1){
//	        return core::kinematics::FoldTree();
//}

//core::kinematics::FoldTree foldtree_ss = protocols::bootcamp::fold_tree_from_ss( *mypose );

int main( int argc, char ** argv ) {
	devel::init( argc, argv );
	utility::vector1< std::string > filenames = basic::options::option[ basic::options::OptionKeys::in::file::s ].value();
	
	//Define the pose and the default score function objects
	core::pose::PoseOP mypose = core::import_pose::pose_from_file( filenames[1] );
	core::scoring::ScoreFunctionOP sfxn = core::scoring::get_score_function();
	
	//Testing the dssp stuff
	
	fold_tree_from_ss(*mypose);

	//Calculate and print the score of the given pose
	core::Real score = sfxn->score( *mypose );
	std::cout << score << std::endl;

	//Create a Monte Carlo constructor mc that takes as input a pose and a score function, plus a temperature
        protocols::moves::MonteCarlo mc(*mypose, *sfxn, 0.8);

	//Create a copy pose object
	core::pose::Pose copy_pose;


	//Define the mc_counter as float and the maximum size of the runs as an integer
	core::Real mc_counter = 0;
	core::Size max = 20;

	for (core::Size i = 0; i < max; i++){
	
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

	//Check the code for the accepteed structure and count if the code is not 0
	core::Size acc_code = mc.mc_accepted();
	if (acc_code != 0){
		mc_counter++;
	}
	
	//Print the acceptance rate every 10 steps
	if ((i + 1)  % 10 == 0){
		core::Real acc_rate = mc_counter / mc.total_trials();
		std::cout << "The acceptance rate is: " << acc_rate << std::endl;
	}


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

	if ( filenames.size() > 0 ) {
		std::cout << "You entered: " << filenames[ 1 ] << " as the PDB file to be read" << std::endl;
	} else {
			std::cout << "You didn’t provide a PDB file with the -in::file::s option" << std::endl;
				return 1;
	}

	std::cout << "Hello World!" << std::endl;
	return 0;
} 

