find_package(Boost COMPONENTS python)
# The python version needs to match the one used to build Boost.Python
find_package(Python 2.6)
find_python_module(pyplusplus)
find_python_module(pygccxml)
find_package(GCCXML)

if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY AND PY_PYPLUSPLUS
	AND PY_PYGCCXML AND GCCXML AND NOT BUILD_PY_OMPL)
	message(STATUS "Building OMPL Python module")
	set(BUILD_PY_OMPL ON CACHE BOOL 
		"Whether the OMPL Python module should be built")
endif()

if(BUILD_PY_OMPL)
	set(OMPL_BASE_HEADERS 
		util/ClassForward.h
		base/State.h
		base/StateSampler.h
		base/StateManifold.h
		base/StateValidityChecker.h
		base/SpaceInformation.h
		base/ScopedState.h
		base/Goal.h
		base/GoalRegion.h
		base/GoalSampleableRegion.h
		base/GoalState.h
		base/GoalStates.h
		base/Planner.h
		base/ProblemDefinition.h
		base/ProjectionEvaluator.h
		base/manifolds/RealVectorStateManifold.h
		base/manifolds/SO2StateManifold.h
		base/manifolds/SO3StateManifold.h
		base/manifolds/SE2StateManifold.h
		base/manifolds/SE3StateManifold.h
		base/manifolds/RealVectorStateProjections.h)
	set(OMPL_CONTROL_HEADERS
		control/Control.h
		control/ControlSampler.h
		control/ControlManifold.h
		control/SpaceInformation.h
		control/PathControl.h
		control/SimpleSetup.h
		control/manifolds/RealVectorControlManifold.h
		control/planners/kpiece/KPIECE1.h
		control/planners/rrt/RRT.h)
	set(OMPL_GEOMETRIC_HEADERS
		geometric/PathGeometric.h
		geometric/PathSimplifier.h
		geometric/SimpleSetup.h
		geometric/planners/est/EST.h
		geometric/planners/kpiece/KPIECE1.h
		geometric/planners/kpiece/LBKPIECE1.h
		geometric/planners/rrt/RRT.h
		geometric/planners/rrt/RRTConnect.h
		geometric/planners/rrt/pRRT.h
		geometric/planners/rrt/LazyRRT.h
		geometric/planners/sbl/SBL.h
		geometric/planners/sbl/pSBL.h)
	set(OMPL_UTIL_HEADERS
		util/Console.h
		util/Exception.h
		util/RandomNumbers.h)
		
	set(OMPL_MODULES base control geometric util)
	set(OMPL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/src")
	include_directories(${PYTHON_INCLUDE_DIRS} "${CMAKE_CURRENT_BINARY_DIR}/bindings" ".")	
	if(APPLE)
		# For some reason gccxml generates 32-bit binaries by default on OS X
		# (maybe because the kernel is still 32-bit?)
		set(PYOMPL_EXTRA_CFLAGS "-m64")
	endif(APPLE)
	
	# need to run cmake to regenerate generate_bindings.py if 
	# generate_bindings.py is changed.
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py.in"
		"generate_bindings.py" @ONLY)

	# target for regenerating code for all python modules
	add_custom_target(update_bindings)

	foreach(module ${OMPL_MODULES})
		string(TOUPPER ${module} module_upper)
		set(module_header "${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}.h")
		file(WRITE "${module_header}" "")
		foreach(header ${OMPL_${module_upper}_HEADERS})
			file(READ "${CMAKE_SOURCE_DIR}/src/ompl/${header}" _header_txt)
			file(APPEND "${module_header}" "${_header_txt}")
		endforeach(header)
		
		# bootstrap code generation
		find_file(BINDINGS_MAIN "_${module}.main.cpp" 
			"${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}" NO_DEFAULT_PATH)
		if(NOT BINDINGS_MAIN)
		 	execute_process(COMMAND ${PYTHON_EXEC} "../generate_bindings.py" "${module}"
		 		WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bindings")
		endif(NOT BINDINGS_MAIN)
		
		# target for regenerating code
		add_custom_target(update_${module}_bindings 
			${PYTHON_EXEC} "../generate_bindings.py" "${module}"
	 		WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bindings")
		add_dependencies(update_bindings update_${module}_bindings)
		
		# target for each python module
		aux_source_directory("${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}" PY${module}BINDINGS)
		add_library(py_ompl_${module} MODULE ${PY${module}BINDINGS})
		target_link_libraries(py_ompl_${module} 
			ompl ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
		add_dependencies(update_${module}_bindings ${OMPL_${module_upper}_HEADERS})
		get_target_property(PY${module}_NAME py_ompl_${module} LOCATION)
		add_custom_command(TARGET py_ompl_${module} POST_BUILD
		        COMMAND cp "${PY${module}_NAME}" 
		        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/${module}/_${module}${CMAKE_SHARED_MODULE_SUFFIX}"
		        WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
    	include_directories("${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}")	
	endforeach(module)
	
	# python unit tests
	add_test(test_py_ompl "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/test.py")
endif(BUILD_PY_OMPL)

