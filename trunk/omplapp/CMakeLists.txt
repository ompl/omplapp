
cmake_minimum_required(VERSION 2.6)
project(ompl CXX)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
	
include(FindPackageHandleStandardArgs)
include(FindPQP)
include(FindAssimp)

find_package(Boost COMPONENTS date_time thread filesystem system REQUIRED)

include_directories("${Boost_INCLUDE_DIR}")
include_directories("${ASSIMP_INCLUDE_DIR}")
include_directories("${PQP_INCLUDE}")
include_directories(ompl/src)

set(APP_LINK_LIBS ompl ${PQP_LIB} ${ASSIMP_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_subdirectory(ompl/src)
add_subdirectory(app)
