cmake_minimum_required(VERSION 2.8)

project(omplapp CXX C)


set(OMPLAPP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPLAPP_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src")
set(OMPL_DEMO_INSTALL_DIR share/ompl/demos
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_DOC_INSTALL_DIR share/ompl/doc
    CACHE STRING "Relative path to directory where documentation will be installed")
set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(CompilerSettings)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include(OMPLVersion)
set(OMPLAPP_MAJOR_VERSION ${OMPL_MAJOR_VERSION})
set(OMPLAPP_MINOR_VERSION ${OMPL_MINOR_VERSION})
set(OMPLAPP_PATCH_VERSION ${OMPL_PATCH_VERSION})
set(OMPLAPP_VERSION "${OMPLAPP_MAJOR_VERSION}.${OMPLAPP_MINOR_VERSION}.${OMPLAPP_PATCH_VERSION}")

find_package(Boost COMPONENTS date_time thread filesystem system python REQUIRED)
find_package(OpenGL REQUIRED)
# This includes our own FindPython.cmake in ompl/CMakeModules. It defines,
# among other things, the find_python_module() function used below.
find_package(Python REQUIRED)
find_python_module(PyQt4 REQUIRED)
# due to a bug in PySide, we can't use it just yet
#find_python_module(PyQt4 QUIET)
#find_python_module(PySide QUIET)
# if(NOT PY_PYQT4 AND NOT PY_PYSIDE)
#     message(SEND_ERROR "Either PyQt4 or PySide needs to be installed.")
# endif()
find_python_module(OpenGL REQUIRED)
find_package(ODE QUIET)
find_package(Drawstuff QUIET)
# The ExternalProject.cmake that comes with CMake 2.8.[01] is broken
# and may not even exist in earlier versions of CMake. We need this
# module to download and build PQP and Assimp (if they aren't already installed).
if(${CMAKE_VERSION} VERSION_LESS 2.8.2 AND
    NOT EXISTS "${CMAKE_SOURCE_DIR}/CMakeModules/ExternalProject.cmake")
    file(DOWNLOAD
            "http://cmake.org/gitweb?p=cmake.git;a=blob_plain;f=Modules/ExternalProject.cmake;hb=HEAD"
            "${CMAKE_SOURCE_DIR}/CMakeModules/ExternalProject.cmake")
endif()
include(UsePQP)
include(UseAssimp)

set(OMPL_EXTENSION_ODE ${ODE_FOUND})
if (ODE_FOUND)
    add_definitions(${ODE_DEFINITIONS})
    include_directories(${ODE_INCLUDE_DIR})
endif()

enable_testing()

include_directories(
    "${Boost_INCLUDE_DIR}"
    "${OPENGL_INCLUDE_DIR}"
    "${ASSIMP_INCLUDE_DIR}"
    "${PQP_INCLUDE_DIR}"
    "${OMPLAPP_INCLUDE_DIR}"
    "${OMPL_INCLUDE_DIR}")

set(OMPLAPP_MODULE_LIBRARIES
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARY}
    ${PQP_LIBRARY}
    ${OMPL_APP_LIBRARY})
set(OMPLAPP_LIBRARIES
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARY}
    ${PQP_LIBRARY})

add_subdirectory(ompl/src)
add_subdirectory(ompl/py-bindings)
add_subdirectory(ompl/tests)
add_subdirectory(ompl/demos)
add_subdirectory(gui)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(demos)
add_subdirectory(py-bindings)
install(DIRECTORY resources DESTINATION share/ompl PATTERN ".svn" EXCLUDE)

# HACK ALERT: just in case the version of ODE used for libdrawstuff is used for libompl
if(TARGET drawstuff)
    add_dependencies(ompl drawstuff)
endif()

if(OMPL_BUILD_PYBINDINGS AND PY_OMPL_GENERATE)
    # need PQP header files to generate python bindings
    if(TARGET pqp)
        add_dependencies(update_app_bindings pqp)
    endif()
    # need assimp header files to generate python bindings
    if(TARGET assimp)
        add_dependencies(update_app_bindings assimp)
    endif()
endif()

include(CPackSettings)
