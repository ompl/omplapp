cmake_minimum_required(VERSION 2.8)

project(omplapp CXX C)

set(OMPLAPP_MAJOR_VERSION 0)
set(OMPLAPP_MINOR_VERSION 9)
set(OMPLAPP_PATCH_VERSION 0)
set(OMPLAPP_VERSION "${OMPLAPP_MAJOR_VERSION}.{$OMPLAPP_MINOR_VERSION}.${OMPLAPP_PATCH_VERSION}")

set(CMAKE_MODULE_PATH 
	"${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
	"${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include(UsePQP)
include(UseAssimp)
find_package(Doxygen)
find_package(Boost COMPONENTS date_time thread filesystem system REQUIRED)

enable_testing()

include_directories(
	"${Boost_INCLUDE_DIR}" 
	"${ASSIMP_INCLUDE_DIR}" 
	"${PQP_INCLUDE_DIR}"
	ompl/src)

set(OMPL_APP_LIBRARIES 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${ASSIMP_LIBRARY} 
	${PQP_LIBRARY}
	ompl)

add_subdirectory(ompl)
add_subdirectory(app)
