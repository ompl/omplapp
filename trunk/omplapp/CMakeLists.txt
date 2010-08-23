cmake_minimum_required(VERSION 2.8)

project(omplapp CXX C)

set(OMPLAPP_MAJOR_VERSION 0)
set(OMPLAPP_MINOR_VERSION 9)
set(OMPLAPP_PATCH_VERSION 0)
set(OMPLAPP_VERSION "${OMPLAPP_MAJOR_VERSION}.{$OMPLAPP_MINOR_VERSION}.${OMPLAPP_PATCH_VERSION}")

set(OMPL_APP_MODULE app)
set(OMPL_APP_LIB ompl_app)
set(OMPL_APP_ARCHIVE "${OMPL_APP_LIB}_static")
set(OMPL_APP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/app")
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
find_package(Doxygen)
find_package(Ruby)
find_package(Boost COMPONENTS date_time thread filesystem system python REQUIRED)
find_package(OpenGL REQUIRED)
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
	${ASSIMP_LIBRARY}_static
	${PQP_LIBRARY}_static)
set(OMPL_APP_LIBRARIES 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${OPENGL_LIBRARIES}
	${ASSIMP_LIBRARY} 
	${PQP_LIBRARY}
	ompl)

add_subdirectory(doc)
add_subdirectory(ompl)
add_subdirectory(app)
