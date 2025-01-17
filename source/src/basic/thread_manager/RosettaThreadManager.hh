// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington CoMotion, email: license@uw.edu.

/// @file basic/thread_manager/RosettaThreadManager
/// @brief A manager that maintains a threadpool and handles requests for threads for multithreaded
/// execution of functions.  This allows multithreading at many different levels in the Rosetta
/// library hierarchy, from job-level parallel execution down to parallel computation of a score,
/// gradient vector, or interaction graph.
/// @note In single-threaded builds, this object still exists.  It accepts vectors of work and executes
/// them directly, in this case.
/// @author Vikram K. Mulligan (vmulligan@flatironinstitute.org)


#ifndef INCLUDED_basic_thread_manager_RosettaThreadManager_hh
#define INCLUDED_basic_thread_manager_RosettaThreadManager_hh

// Unit headers
#include <basic/thread_manager/RosettaThreadManager.fwd.hh>
#include <basic/thread_manager/RosettaThreadAssignmentInfo.fwd.hh>

#ifdef MULTI_THREADED
#include <basic/thread_manager/RosettaThreadManagerInitializationTracker.hh>
#include <basic/thread_manager/RosettaThreadPool.fwd.hh>
#endif

#include <basic/thread_manager/RosettaThreadManagerAdvancedAPIKey.fwd.hh>
#include <basic/thread_manager/RosettaThreadAllocation.hh>

// Utility header
#include <utility/SingletonBase.hh>
#include <utility/vector1.hh>

// Platform headers
#include <platform/types.hh>

// C++ header
#ifdef MULTI_THREADED
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <deque>
#endif

namespace basic {
namespace thread_manager {

#ifdef MULTI_THREADED
/// @brief A container for a std::atomic_bool.
/// @details The std::atomic_bool class has no copy constructor or initialization constructor, so it can't be encaspulated in a
/// vector if the vector is ever resized, used with push_back, etc.  For most applications, this is fine.  With older compilers
/// and older versions of the STL (e.g. clang 3.4 and gcc 4.8.2's STL), even constructing the fixed-size vector is problematic,
/// though.
class AtomicBoolContainer {
public:

	/// @brief Default constructor -- initializes to "false".
	AtomicBoolContainer() {
		contained_bool_ = false;
	}

	/// @brief Copy constructor.
	AtomicBoolContainer( AtomicBoolContainer const &src ) {
		contained_bool_ = src.contained_bool_.load();
	}

	/// @brief Assignment operator.
	AtomicBoolContainer &
	operator=( AtomicBoolContainer const &other ) {
		if ( this == &other ) {
			return *this;
		}

		contained_bool_ = other.contained_bool_.load();
		return *this;
	}

public:

	std::atomic_bool contained_bool_;
};
#endif

/// @brief A manager that maintains a threadpool and handles requests for threads for multithreaded execution of functions.  This allows
/// multithreading at many different levels in the Rosetta library hierarchy, from job-level parallel execution down to parallel
/// computation of a score, gradient vector, or interaction graph.
/// @note In single-threaded builds, this object still exists.  It accepts vectors of work and executes them directly, in this case.
class RosettaThreadManager : public utility::SingletonBase< RosettaThreadManager > {
	friend class utility::SingletonBase< RosettaThreadManager >;

private:  // Private methods //////////////////////////////////////////////////

	/// @brief Empty constructor.
	RosettaThreadManager();

	/// @brief Copy constructor -- explicitly deleted.
	RosettaThreadManager(RosettaThreadManager const & ) = delete;

	/// @brief Assignment operator -- explicitly deleted.
	RosettaThreadManager operator=(RosettaThreadManager const & ) = delete;

	/// @brief Destructor.  Non-empty, since threads must be spun down.
	~RosettaThreadManager();

#ifdef MULTI_THREADED
	/// @brief Creates the thread pool if it has not yet been created.  Safe to call repeatedly.  The
	/// thread_pool_mutex_ must be locked before calling this function!
	/// @details Accesses the global options system to determine the number of threads to launch.  Also, registers the
	/// RosettaThreadManager with the CitationManager if launching threads.
	void create_thread_pool();

	/// @brief Adds citation information for the RosettaThreadManager to the CitationManager.
	/// @details The thread_pool_mutex_ must be locked before calling this function!
	void register_thread_manager_with_citation_manager() const;
#endif

public:  // Public methods ////////////////////////////////////////////////////

#ifdef MULTI_THREADED

	/// @brief Trigger launch of threads.
	/// @details Does nothing if threads already launched.
	void launch_threads();

#endif

	/// @brief Get the total number of threads that have been launched or will be launched.
	/// @details This corresponds to the -multithreading:total_threads option, but it does not access the options system repatedly.d
	/// @note Always returns 1 in the non-threaded build.
	inline static platform::Size total_threads() {
#ifdef MULTI_THREADED
		return RosettaThreadManagerInitializationTracker::get_instance()->total_threads();
#else
		return 1;
#endif
	}

#ifdef MULTI_THREADED
	/// @brief Report whether the RosettaThreadManager was initialized (i.e. whether threads have been launched).
	inline static bool thread_manager_was_initialized() { return RosettaThreadManagerInitializationTracker::get_instance()->thread_manager_was_initialized(); }

	/// @brief Report whether the RosettaThreadManager initialization has begun (i.e. whether threads have been launched OR are in the process of being launched).
	inline static bool thread_manager_initialization_begun() { return RosettaThreadManagerInitializationTracker::get_instance()->thread_manager_initialization_begun(); }
#endif

	/// @brief BASIC API THAT SHOULD BE USED IN MOST CIRCUMSTANCES.  Given a vector of functions that were bundled with
	/// their arguments with std::bind, each of which can be executed in any order and each of which is safe to execute
	/// in parallel with any other, run all of these in threads.
	/// @details The bundled functions should be atomistic pieces of work.  They should be bundled with their arguments with
	/// std::bind, and the arguments should include the place to store output (i.e. they should return void).  These functions
	/// should not handle any mutexes themselves, but should ensure that they are operating only on memory locations that no
	/// other functions in the vector are operating on.
	/// @note Under the hood, this sets up appropriate mutexes and then calls run_function_in_threads() to do the work.  The work
	/// is done concurrently in 1 <= actual count <= min( requested thread count, total thread count ) threads.  The function blocks
	/// until all threads have finished their work, which means that the individual work units should be small, that the longest-running
	/// work unit should be short compared to the total runtime, and that the number of work units should be much greater than the
	/// number of threads requested.
	void
	do_work_vector_in_threads(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		platform::Size const requested_thread_count,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief BASIC API THAT SHOULD BE USED IN MOST CIRCUMSTANCES.  Given a vector of functions that were bundled with
	/// their arguments with std::bind, each of which can be executed in any order and each of which is safe to execute
	/// in parallel with any other, run all of these in threads.
	/// @details The bundled functions should be atomistic pieces of work.  They should be bundled with their arguments with
	/// std::bind, and the arguments should include the place to store output (i.e. they should return void).  These functions
	/// should not handle any mutexes themselves, but should ensure that they are operating only on memory locations that no
	/// other functions in the vector are operating on.
	/// @note Under the hood, this sets up appropriate mutexes and then calls run_function_in_threads() to do the work.  The work
	/// is done concurrently in 1 <= actual count <= min( requested thread count, total thread count ) threads.  The function blocks
	/// until all threads have finished their work, which means that the individual work units should be small, that the longest-running
	/// work unit should be short compared to the total runtime, and that the number of work units should be much greater than the
	/// number of threads requested.
	void
	do_work_vector_in_threads(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		RosettaThreadAllocation & allocation,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief VARIANT BASIC API THAT SHOULD BE USED FOR WORK VECTORS OF NEAR-EQUAL SIZED CHUNKS WHERE THE CHUNKS ARE SMALL.  Given a vector of
	/// functions that were bundled with their arguments with std::bind, each of which can be executed in any order and each of which is safe to execute
	/// in parallel with any other, run all of these in threads.
	/// @details The bundled functions should be atomistic pieces of work.  They should be bundled with their arguments with
	/// std::bind, and the arguments should include the place to store output (i.e. they should return void).  These functions
	/// should not handle any mutexes themselves, but should ensure that they are operating only on memory locations that no
	/// other functions in the vector are operating on.
	/// @note Under the hood, this sets up no mutexes, instead giving each thread a staggered subset of the work in the vector.  It calls
	/// run_function_in_threads() to do the work.  The work is done concurrently in 1 <= actual count <= min( requested thread count, total
	/// thread count ) threads.  The function blocks until all threads have finished their work, which means that the individual work units
	/// should be small, that the longest-running work unit should be short compared to the total runtime, and that the number of work units
	/// should be much greater than the number of threads requested.
	/// This function works best for cases in which it is known that most of the work in the vector is of equal size (i.e. load-balancing is
	/// unlikely to be an issue), and where the overhead of locking mutexes for each job is likely to be comparable in size to the cost of a
	/// job (so we want to avoid this overhead).
	void
	do_work_vector_in_threads_no_locking(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		platform::Size const requested_thread_count,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief VARIANT BASIC API THAT SHOULD BE USED FOR WORK VECTORS OF NEAR-EQUAL SIZED CHUNKS WHERE THE CHUNKS ARE SMALL.  Given a vector of
	/// functions that were bundled with their arguments with std::bind, each of which can be executed in any order and each of which is safe to execute
	/// in parallel with any other, run all of these in threads.
	/// @details The bundled functions should be atomistic pieces of work.  They should be bundled with their arguments with
	/// std::bind, and the arguments should include the place to store output (i.e. they should return void).  These functions
	/// should not handle any mutexes themselves, but should ensure that they are operating only on memory locations that no
	/// other functions in the vector are operating on.
	/// @note Under the hood, this sets up no mutexes, instead giving each thread a staggered subset of the work in the vector.  It calls
	/// run_function_in_threads() to do the work.  The work is done concurrently in 1 <= actual count <= min( requested thread count, total
	/// thread count ) threads.  The function blocks until all threads have finished their work, which means that the individual work units
	/// should be small, that the longest-running work unit should be short compared to the total runtime, and that the number of work units
	/// should be much greater than the number of threads requested.
	/// This function works best for cases in which it is known that most of the work in the vector is of equal size (i.e. load-balancing is
	/// unlikely to be an issue), and where the overhead of locking mutexes for each job is likely to be comparable in size to the cost of a
	/// job (so we want to avoid this overhead).
	void
	do_work_vector_in_threads_no_locking(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		RosettaThreadAllocation & allocation,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief VARIANT BASIC API THAT SHOULD BE USED WHERE THE BASIC API CAN'T BE USED.  Given a vector of vectors of functions
	/// that were bundled with their arguments with std::bind, run all of these in threads.  In this case, the bundled functions
	/// are in groups, where the individual functions within a group can run in any order (and are safe to run concurrently),
	/// but the groups must be run sequentially.  This is useful when, for example, you have a bunch of calculations to do, and
	/// then some finalization tasks to do after the calculations are done, and you don't want to re-request threads.
	/// @details The bundled functions should be atomistic pieces of work.  They should be bundled with their arguments with
	/// std::bind, and the arguments should include the place to store output (i.e. they should return void).  These functions
	/// should not handle any mutexes themselves, but should ensure that they are operating only on memory locations that no
	/// other functions in the vector are operating on.
	/// @note Under the hood, this sets up appropriate mutexes and then calls run_function_in_threads() to do the work.  The work
	/// is done concurrently in 1 <= actual count <= min( requested thread count, total thread count ) threads.  The function blocks
	/// until all threads have finished their work, which means that the individual work units should be small, that the longest-running
	/// work unit should be short compared to the total runtime, and that the number of work units should be much greater than the
	/// number of threads requested.
	void
	do_multistage_work_vector_in_threads(
		utility::vector1< utility::vector1< RosettaThreadFunction > > const & multistage_vector_of_work,
		platform::Size const requested_thread_count,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief ADVANCED API THAT SHOULD NOT BE USED IN MOST CIRCUMSTANCES.  Given a function that was bundled with its
	/// arguments with std::bind, run it in many threads.  This calls RosettaThreadPool::run_function_in_threads for
	/// the already-running thread pool.  If the thread pool has not been created, it first creates it by calling
	/// create_thread_pool().  IF YOU DECIDE TO USE THE ADVANCED API, YOU MUST:
	/// 1. Pass this function a RosettaThreadManagerAdvancedAPIKey from the calling context.  Since
	/// the RosettaThreadManagerAdvancedAPIKey class has a private constructor, it can only be created in
	/// whitelisted contexts in its friend list, which means that you must:
	/// 2. Add the class that calls this advanced API to the friend list for the RosettaThreadManagerAdvancedAPIKey
	/// class.  Since this will trigger breakage of the central_class_modification regression test, you must finally:
	/// 3. Justify to the developer community why you must call this interface and not the safer, basic interface (do_work_vector_in_threads)
	/// in both the comments in RosettaThreadManagerAdvancedAPIKey's friend list, the comments in the calling class, AND in your
	/// pull request description.  Andrew Leaver-Fay and Vikram K. Mulligan will both scrutinize this closely.  It is highly recommended
	/// that before using the run_function_in_threads() function, you first contact Andrew or Vikram and discuss whether it is possible
	/// to do what you want to do using the basic API (the do_work_vector_in_threads() function).
	///
	/// @details The function is assigned to as many threads as the RosettaThreadPool decides to assign it to, always
	/// including the thread from which the request originates.  It is guaranteed to run in 1 <= actual_thread_count <=
	/// requested_thread_count threads.  After assigning the function to up to (requsted_thread_count - 1) other threads,
	/// the function executes in the current thread, then the current thread blocks until the assigned threads report that
	/// they are idle.  All of this is handled by the RosettaThreadPool class (or its derived classes, which may have)
	/// different logic for assigning thread requests to threads).
	///
	/// @note A RosettaThreadAssignmentInfo object should be passed in.  It will be populated with
	/// the number of threads requested, the number actually assigned, the indices of the assigned threads, and a map of
	/// system thread ID to Rosetta thread index.  The same owning pointer may optionally be provided to the function to
	/// execute by the calling function if the function to execute requires access to this information.  Note also that the
	/// function passed in is responsible for ensuring that it is able to carry out a large block of work, alone or concurrently
	/// with many copies of itself in parallel threads, in a threadsafe manner.  Finally, note that this function requires a
	/// RosettaThreadManagerAdvancedAPIKey, which can only be instantiated by friend classes in the whitelist in the
	/// RosettaThreadManagerAdvancedAPIKey class definition.  This ensures that only select classes can access the advanced
	/// RosettaThreadManager API.
	void
	run_function_in_threads(
		RosettaThreadFunction & function_to_execute,
		platform::Size const requested_thread_count,
		RosettaThreadManagerAdvancedAPIKey const & key,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	/// @brief ADVANCED API THAT SHOULD NOT BE USED IN MOST CIRCUMSTANCES.  Given a function that was bundled with its
	/// arguments with std::bind, run it in many threads.  This calls RosettaThreadPool::run_function_in_threads for
	/// the already-running thread pool.  If the thread pool has not been created, it first creates it by calling
	/// create_thread_pool().  IF YOU DECIDE TO USE THE ADVANCED API, YOU MUST:
	/// 1. Pass this function a RosettaThreadManagerAdvancedAPIKey from the calling context.  Since
	/// the RosettaThreadManagerAdvancedAPIKey class has a private constructor, it can only be created in
	/// whitelisted contexts in its friend list, which means that you must:
	/// 2. Add the class that calls this advanced API to the friend list for the RosettaThreadManagerAdvancedAPIKey
	/// class.  Since this will trigger breakage of the central_class_modification regression test, you must finally:
	/// 3. Justify to the developer community why you must call this interface and not the safer, basic interface (do_work_vector_in_threads)
	/// in both the comments in RosettaThreadManagerAdvancedAPIKey's friend list, the comments in the calling class, AND in your
	/// pull request description.  Andrew Leaver-Fay and Vikram K. Mulligan will both scrutinize this closely.  It is highly recommended
	/// that before using the run_function_in_threads() function, you first contact Andrew or Vikram and discuss whether it is possible
	/// to do what you want to do using the basic API (the do_work_vector_in_threads() function).
	///
	/// @details The function is assigned to as many threads as the RosettaThreadPool decides to assign it to, always
	/// including the thread from which the request originates.  It is guaranteed to run in 1 <= actual_thread_count <=
	/// requested_thread_count threads.  After assigning the function to up to (requsted_thread_count - 1) other threads,
	/// the function executes in the current thread, then the current thread blocks until the assigned threads report that
	/// they are idle.  All of this is handled by the RosettaThreadPool class (or its derived classes, which may have)
	/// different logic for assigning thread requests to threads).
	///
	/// @note A RosettaThreadAssignmentInfo object should be passed in.  It will be populated with
	/// the number of threads requested, the number actually assigned, the indices of the assigned threads, and a map of
	/// system thread ID to Rosetta thread index.  The same owning pointer may optionally be provided to the function to
	/// execute by the calling function if the function to execute requires access to this information.  Note also that the
	/// function passed in is responsible for ensuring that it is able to carry out a large block of work, alone or concurrently
	/// with many copies of itself in parallel threads, in a threadsafe manner.  Finally, note that this function requires a
	/// RosettaThreadManagerAdvancedAPIKey, which can only be instantiated by friend classes in the whitelist in the
	/// RosettaThreadManagerAdvancedAPIKey class definition.  This ensures that only select classes can access the advanced
	/// RosettaThreadManager API.
	void
	run_function_in_threads(
		RosettaThreadFunction & function_to_execute,
		RosettaThreadManagerAdvancedAPIKey const & key,
		RosettaThreadAllocation & allocation
	);

	/// @brief Get the Rosetta thread index.
	platform::Size get_rosetta_thread_index() const;

public: //Preallocation Section

	///@brief Allocate threads before supplying the work vector
	RosettaThreadAllocation
	reserve_threads(
		platform::Size const requested_thread_count,
		RosettaThreadAssignmentInfo & thread_assignment
	);

	///@breif deallocate threads given by reserve_threads()
	///@details This is called by RosettaThreadAllocation's destructor, so you don't need to worry about it
	void
	release_threads(
		RosettaThreadAllocation & allocation
	);

private:  // Private fxns /////////////////////////////////////////////////////
#ifdef MULTI_THREADED

	/// @brief The function that is passed by do_work_vector_in_threads() to run_function_in_threads() to run in parallel,
	/// to execute a vector of work in a threadsafe manner.
	void
	work_vector_thread_function(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		utility::vector1< std::mutex > & job_mutexes,
		utility::vector1< AtomicBoolContainer > & jobs_completed,
		RosettaThreadAssignmentInfo const & thread_assignments
	) const;

	/// @brief The function that is passed by do_work_vector_in_threads_no_locking() to run_function_in_threads() to run in parallel,
	/// to execute a vector of work in a threadsafe manner, without locking each task.
	/// @details This version assigns every Nth piece of work to a given thread.  The assumption is that this will result in even load-balancing
	/// without the overhead of locking.  This is true if the pieces of work are of roughly equal size.
	void
	work_vector_thread_function_no_locking(
		utility::vector1< RosettaThreadFunction > const & vector_of_work,
		RosettaThreadAssignmentInfo const & thread_assignments
	) const;

	/// @brief The function that is passed by do_multistage_work_vector_in_threads() to run_function_in_threads() to run in parallel,
	/// to execute a vector of work in a threadsafe manner.
	void
	multistage_work_vector_thread_function(
		utility::vector1< utility::vector1< RosettaThreadFunction > > const & multistage_vector_of_work,
		std::deque< std::deque< std::mutex > > & multistage_job_mutexes,
		std::deque< std::deque< AtomicBoolContainer > > & multistage_jobs_completed,
		std::mutex & barrier_mutex,
		utility::vector1< platform::Size > & barrier_threadcount,
		std::condition_variable & barrier_cv,
		RosettaThreadAssignmentInfo & thread_assignment
	) const;

#endif

private:  // Private data /////////////////////////////////////////////////////

#ifdef MULTI_THREADED

	/// @brief The number of threads to launch.
	/// @details Read from options system.  If the options system specifies "0",
	/// this is set to the number of cores available on the current node.
	platform::Size nthreads_ = 0;

	/// @brief The pool of always-running threads that we always manage.  Created on the first call to
	/// run_function_in_threads().
	RosettaThreadPoolOP thread_pool_ = nullptr;

	/// @brief A mutex for locking the thread pool during creation.
	mutable std::mutex thread_pool_mutex_;

	/// @brief Was the thread pool launched?  Cheaper than locking a mutex to check the pointer.
	mutable std::atomic_bool thread_pool_was_launched_;

	/// @brief Map of system thread ID to Rosetta thread index.
	std::map< std::thread::id, platform::Size > thread_id_to_rosetta_thread_index_;

	/// @brief A counter for the number of times the invalid thread warning has been issued.  Used to prevent
	/// runaway unnecessary output.
	mutable std::atomic_int warning_counter_;

	/// @brief A mutex for the warning counter.
	mutable std::mutex warning_counter_mutex_;

#endif

};

} //thread_manager
} //basic

#endif //INCLUDED_basic/thread_manager_RosettaThreadManager_fwd_hh

