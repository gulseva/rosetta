// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington UW TechTransfer, email: license@u.washington.edu.

/// @file   protocols/jd3/pose_inputters/JD2ResourceManager.cxxtest.hh
/// @brief  test suite for protocols::jd2::JD2ResourceManager and protocols::resource_manager::LazyResourceManager
/// @author Andrew Leaver-Fay (aleaverfay@gmail.com)

// Test headers
#include <cxxtest/TestSuite.h>
#include <test/protocols/init_util.hh>

// Unit headers
#include <protocols/jd3/pose_inputters/PDBPoseInputter.hh>
#include <protocols/jd3/PoseInputSource.hh>


template <typename T>
typename T::mapped_type get(T const& map, typename T::key_type const& key)
{
	typename T::const_iterator iter(map.find(key));
	return iter != map.end() ? iter->second : typename T::mapped_type();
}

using protocols::jd3::PoseInputSources;
using protocols::jd3::PoseInputSourceOP;
using protocols::jd3::pose_inputters::PDBPoseInputter;

class JD3PDBPoseInputterTests : public CxxTest::TestSuite {
public:

	void setUp() {
	}

	void test_read_s_flag() {
		protocols_init_with_additional_options( "-s /home/andrew/1ubq.pdb" );
		PDBPoseInputter inputter;
		PoseInputSources sources = inputter.initialize_pose_input_sources();
		TS_ASSERT_EQUALS( sources.size(), 1 );
		if ( sources.size() != 1 ) return;
		TS_ASSERT_EQUALS( sources[1]->input_kind(), protocols::jd3::pik_pdb_file );
		TS_ASSERT_EQUALS( sources[1]->input_tag(), std::string("1ubq") );
		TS_ASSERT_EQUALS( sources[1]->origin(), protocols::jd3::piso_command_line );
		TS_ASSERT( sources[1]->string_string_map().find( std::string("filename") ) != sources[1]->string_string_map().end() );
		TS_ASSERT_EQUALS( get( sources[1]->string_string_map(), std::string("filename")), "/home/andrew/1ubq.pdb" );
	}
};