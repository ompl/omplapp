option(ROS "Build against ROS" FALSE)

if(ROS)
	rosbuild_add_boost_directories()
else(ROS)
	cmake_minimum_required(VERSION 2.8)
	project(ompl CXX)
	set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
	set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

	include(FindPackageHandleStandardArgs)
	find_package(Boost COMPONENTS date_time thread filesystem system REQUIRED)
	include_directories("${Boost_INCLUDE_DIR}")
	if(CMAKE_COMPILER_IS_GNUCXX)
		add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers)
	endif(CMAKE_COMPILER_IS_GNUCXX)
	
	find_package(Doxygen)
	
	enable_testing()
endif(ROS)

include_directories(src)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(py-bindings)
add_subdirectory(tests)
add_subdirectory(demos)
