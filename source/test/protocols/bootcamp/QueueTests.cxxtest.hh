// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file  ././//s/r/c///a/p/p/s///p/i/l/o/t///g/u/l/s/e/v/i/n///Q/u/e/u/e/T/e/s/t/s/./c/x/x/t/e/s/t/./h/h/QueueTests.cxxtest.hh
/// @brief  Unit test for the bootcamp test function
/// @author gulseva (alican@gulsevinlab.org)


// Test headers
#include <test/UMoverTest.hh>
#include <test/UTracer.hh>
#include <cxxtest/TestSuite.h>
#include <test/util/pose_funcs.hh>
#include <test/core/init_util.hh>

// Project Headers
#include <test/protocols/bootcamp/Queue.hh>

// Core Headers
#include <core/pose/Pose.hh>
#include <core/import_pose/import_pose.hh>

// Utility, etc Headers
#include <basic/Tracer.hh>

static basic::Tracer TR("QueueTests");


class QueueTests : public CxxTest::TestSuite {
	//Define Variables

public:

	void setUp() {
		core_init();
		
		//Create a legit list
		queue1 = protocols::bootcamp::Queue();
		queue1.enqueue("erik");
		queue1.enqueue("kalem");
		queue1.enqueue("pil");
		queue1.enqueue("armut");

		//Create an empty list
        	queue2 = protocols::bootcamp::Queue();
		queue5 = queue2;


		//Create a list where the first and last members are the same
                queue3 = protocols::bootcamp::Queue();
		queue3.enqueue("erik");
		queue3.enqueue("kalem");
		queue3.enqueue("erik");

		//Create a list with a single item
	        queue4 = protocols::bootcamp::Queue();
                queue4.enqueue("erik");	

	}

	void tearDown() {

	}



	void test_first() {
		TS_TRACE( "Running my first unit test!" );
              	TS_ASSERT( true );


	}
	
	void test_enqueue(){
		//Check if the queue is empty
		TS_ASSERT(queue1.is_empty() != right);
		TS_ASSERT(queue2.is_empty() != right);
		TS_ASSERT(queue3.is_empty() != right);
                TS_ASSERT(queue4.is_empty() != right);
		TS_ASSERT(queue5.is_empty() != right);
		
		TS_ASSERT_THROWS_ANYTHING(queue5.dequeue());	
	}

private:
	protocols::bootcamp::Queue queue1;
	protocols::bootcamp::Queue queue2;
	protocols::bootcamp::Queue queue3;
	protocols::bootcamp::Queue queue4;
        protocols::bootcamp::Queue queue5;

	bool right = true;

};
