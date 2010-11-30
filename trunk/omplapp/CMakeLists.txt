cmake_minimum_required(VERSION 2.8)

project(omplapp CXX C)

set(OMPLAPP_MAJOR_VERSION 0)
set(OMPLAPP_MINOR_VERSION 9)
set(OMPLAPP_PATCH_VERSION 0)
set(OMPLAPP_VERSION "${OMPLAPP_MAJOR_VERSION}.{$OMPLAPP_MINOR_VERSION}.${OMPLAPP_PATCH_VERSION}")

set(OMPL_APP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src")
set(OMPL_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")

set(CMAKE_MODULE_PATH 
	"${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
	"${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(CompilerSettings)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include(UsePQP)
include(UseAssimp)
include(UseDrawstuff)
find_package(Boost COMPONENTS date_time thread filesystem system python REQUIRED)
find_package(OpenGL REQUIRED)
# This includes our own FindPython.cmake in ompl/CMakeModules. It defines, 
# among other things, the find_python_module() used below.
find_package(Python REQUIRED)
find_python_module(PyQt4 REQUIRED)
find_python_module(OpenGL REQUIRED)

enable_testing()

include_directories(
	"${Boost_INCLUDE_DIR}"
	"${OPENGL_INCLUDE_DIR}"
	"${ASSIMP_INCLUDE_DIR}" 
	"${PQP_INCLUDE_DIR}"
	"${OMPL_APP_INCLUDE_DIR}"
	"${OMPL_INCLUDE_DIR}")

set(OMPL_APP_MODULE_LIBRARIES
	${OPENGL_LIBRARIES}
	${ASSIMP_LIBRARY}
	${PQP_LIBRARY}
	${OMPL_APP_LIBRARY})
set(OMPL_APP_LIBRARIES 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${OPENGL_LIBRARIES}
	${ASSIMP_LIBRARY} 
	${PQP_LIBRARY})

# disable building separate documentation for the OMPL core;
# this documentation is already automatically included in the OMPL.app
# documentation
option(OMPL_BUILD_DOC "Build OMPL documentation" OFF)

# add ompl directory first; in it some options / variables are declared 
# that are referenced by CMakeLists.txt files in other subdirectories.
add_subdirectory(ompl)
add_subdirectory(app)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(demos)
add_subdirectory(py-bindings)
install(DIRECTORY resources DESTINATION share/ompl/resources)

include(CPackSettings)
