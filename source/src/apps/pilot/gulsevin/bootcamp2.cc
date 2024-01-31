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
#include </data/programs/Rosetta/rosetta/source/src/protocols/moves/MoverStatistics.hh>
#include </data/programs/Rosetta/rosetta/source/src/protocols/moves/DsspMover.cc>

//Function 1
core::kinematics::FoldTree fold_tree_from_ss(core::Pose::Pose Pose){
        return core::kinematics::FoldTree();
}

//Function 2
core::kinematics::FoldTree fold_tree_from_ss(std::string input1){
                return core::kinematics::FoldTree();
}


core::pose::PoseOP mypose = core::import_pose::pose_from_file( filenames[1] );


protocols::moves::DsspMover mover;
mover.apply(*mypose);


