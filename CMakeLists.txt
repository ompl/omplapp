cmake_minimum_required(VERSION 2.8)
if(${CMAKE_VERSION} VERSION_GREATER 2.8.3)
    cmake_policy(SET CMP0017 NEW)
endif()
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 OLD)
endif()
project(omplapp CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

# add "${prefix}/share/cmake/Modules", which is where Eigen3's module is in MacPorts.
if(${CMAKE_VERSION} VERSION_LESS 2.8.12)
    get_filename_component(CMAKE_ROOT_DIR "${CMAKE_ROOT}" PATH)
else()
    get_filename_component(CMAKE_ROOT_DIR "${CMAKE_ROOT}" DIRECTORY)
endif()
set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(GNUInstallDirs)
include(CompilerSettings)
include(OMPLVersion)
include(OMPLUtils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(OMPLAPP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPLAPP_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src")
set(OMPL_DEMO_INSTALL_DIR "share/ompl${OMPL_INSTALL_SUFFIX}/demos"
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")

set(OMPLAPP_MAJOR_VERSION ${OMPL_MAJOR_VERSION})
set(OMPLAPP_MINOR_VERSION ${OMPL_MINOR_VERSION})
set(OMPLAPP_PATCH_VERSION ${OMPL_PATCH_VERSION})
set(OMPLAPP_VERSION "${OMPLAPP_MAJOR_VERSION}.${OMPLAPP_MINOR_VERSION}.${OMPLAPP_PATCH_VERSION}")
set(OMPLAPP_ABI_VERSION ${OMPL_ABI_VERSION})

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
if(IS_ICPC)
    set(Boost_USE_STATIC_LIBS ON CACHE STRING "Use statically linked Boost libraries")
else(IS_ICPC)
    # Ensure dynamic linking with boost unit_test_framework
    add_definitions(-DBOOST_TEST_DYN_LINK)
endif(IS_ICPC)

find_package(Boost 1.54 COMPONENTS serialization filesystem system program_options unit_test_framework REQUIRED)

# on OS X we need to check whether to use libc++ or libstdc++ with clang++
if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    include(GetPrerequisites)
    get_prerequisites("${Boost_SYSTEM_LIBRARY}" _libs 0 0 "/" "")
    set(CXXSTDLIB "")
    foreach(_lib ${_libs})
        if(_lib MATCHES "libc\\+\\+")
            set(CXXSTDLIB "libc++")
        elseif(_lib MATCHES "libstdc\\+\\+")
            set(CXXSTDLIB "libstdc++")
        endif()
    endforeach()
    if(CXXSTDLIB)
        add_definitions(-stdlib=${CXXSTDLIB})
    endif()
endif()

find_package(OpenGL)
# This includes our own FindPython.cmake in ompl/CMakeModules. It defines,
# among other things, the find_python_module() function used below.
find_package(Python)
find_package(Boost COMPONENTS python QUIET)
find_python_module(PyQt4 QUIET)
find_python_module(PyQt5 QUIET)
find_python_module(PySide QUIET)
if(NOT (PY_PYQT4 OR PY_PYQT5 OR PY_PYSIDE))
    message(WARNING "Either PyQt4, PyQt5 or PySide needs to be installed to use the GUI.")
endif()
find_python_module(OpenGL)
if (NOT OPENGL_FOUND OR NOT PY_OPENGL)
    message(WARNING "Both OpenGL and the Python OpenGL module need to be installed to use the GUI")
endif()
# Eigen is needed for the InformedStateSampler
find_package(Eigen3 QUIET)
# If Triangle is installed, enable code for triangular decompositions
find_package(Triangle QUIET)
# If FLANN is installed, a wrapper for its nearest neighbor data structures can be used
find_package(flann 1.8.3 QUIET)
# MORSE is only needed for Modular OpenRobots Simulation Engine bindings
find_package(MORSE QUIET)
# ODE is only needed for Open Dynamics Engine bindings
find_package(OpenDE QUIET)
# This library is included in the Open Dynamics source distribution and is used in one demo only
find_package(Drawstuff QUIET)
# Assimp is need to load meshes
find_package(Assimp REQUIRED)
# We need FCL and its dependency CCD for collision checking
find_package(CCD REQUIRED)
find_package(FCL REQUIRED)

# pthread is sometimes needed, depending on OS / compiler
find_package(Threads QUIET)

enable_testing()

include_directories(
    "${OMPLAPP_INCLUDE_DIR}"
    "${OMPL_INCLUDE_DIR}"
    "${Boost_INCLUDE_DIR}"
    "${FCL_INCLUDE_DIRS}"
    "${CCD_INCLUDE_DIRS}"
    "${ASSIMP_INCLUDE_DIRS}")

set(OMPLAPP_MODULE_LIBRARIES
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARY}
    ${FCL_LIBRARIES})
set(OMPLAPP_LIBRARIES
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${CCD_LIBRARIES})
link_directories(${ASSIMP_LIBRARY_DIRS} ${FCL_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})

if(OPENGL_INCLUDE_DIR)
    include_directories("${OPENGL_INCLUDE_DIR}")
endif()

set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})
if (TRIANGLE_FOUND)
  include_directories(${TRIANGLE_INCLUDE_DIR})
endif()

if (EIGEN3_FOUND)
    set(OMPL_HAVE_EIGEN3 1)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif()

if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
    include_directories("${FLANN_INCLUDE_DIRS}")
endif()

set(OMPL_EXTENSION_MORSE ${MORSE_FOUND})

set(OMPL_EXTENSION_OPENDE ${OPENDE_FOUND})
if (OPENDE_FOUND)
    add_definitions(${OPENDE_DEFINITIONS})
    include_directories("${OPENDE_INCLUDE_DIR}")
endif()

option(OMPLAPP_PQP "Enable support for the PQP collision checking library" ON)
if(OMPLAPP_PQP)
    # If the PQP is installed, it is possible to use as the collision checker with OMPL.app.
    # (FCL is the default collision checker)
    find_package(PQP QUIET)
    if(PQP_FOUND)
        include_directories("${PQP_INCLUDE_DIR}")
        set(OMPLAPP_MODULE_LIBRARIES ${OMPLAPP_MODULE_LIBRARIES} ${PQP_LIBRARY})
        set(OMPLAPP_LIBRARIES ${OMPLAPP_LIBRARIES} ${PQP_LIBRARY})
    endif()
endif()

# extra menu option for OMPLapp documentation:
set(OMPLAPP_MENU "<li><a href=\"gui.html\">OMPL.app GUI</a></li><li><a href=\"webapp.html\">OMPL web app</a></li>")
add_subdirectory(ompl/doc)
add_subdirectory(ompl/py-bindings)
add_subdirectory(ompl/src)
add_subdirectory(ompl/tests)
add_subdirectory(ompl/demos)
add_subdirectory(ompl/scripts)
add_subdirectory(gui)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(demos)
add_subdirectory(benchmark)
add_subdirectory(py-bindings)

install(DIRECTORY resources
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT omplapp
    PATTERN ".DS_Store" EXCLUDE)

install(DIRECTORY webapp
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT omplapp
    PATTERN ".DS_Store" EXCLUDE)
find_python_module(flask)
find_python_module(celery)
find_program(CELERY celery)
if(PY_FLASK AND PY_CELERY AND CELERY)
    install(PROGRAMS webapp/ompl_webapp
        DESTINATION bin
        RENAME ompl_webapp${OMPL_INSTALL_SUFFIX})
endif()

# HACK ALERT: just in case the version of OpenDE used for libdrawstuff is used for libompl
if(TARGET drawstuff)
    add_dependencies(ompl drawstuff)
endif()

if(OMPL_BUILD_PYBINDINGS AND PY_OMPL_GENERATE)
    # need assimp header files to generate python bindings
    if(TARGET assimp)
        add_dependencies(update_app_bindings assimp)
    endif()
    # need fcl header files to generate python bindings
    if(TARGET fcl)
        add_dependencies(update_app_bindings fcl)
    endif()
endif()


target_link_flags(ompl ompl_app_base ompl_app)
set(PKG_NAME "ompl")
set(PKG_DESC "The Open Motion Planning Library")
set(PKG_EXTERNAL_DEPS "${ompl_PKG_DEPS}")
set(PKG_OMPL_LIBS "-lompl -lompl_app_base -lompl_app ${ompl_LINK_FLAGS}")
set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/ompl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
install(FILES "${pkg_conf_file}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
    COMPONENT ompl
    RENAME "ompl${OMPL_INSTALL_SUFFIX}.pc")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ompl/doc/markdown/FindOMPL.cmake"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT ompl
    RENAME ompl-config.cmake)
if (NOT ${CMAKE_VERSION} VERSION_LESS 2.8.6)
    include(WriteBasicConfigVersionFile)
    write_basic_config_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/ompl-config-version.cmake"
        VERSION ${OMPL_VERSION} COMPATIBILITY SameMajorVersion)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ompl-config-version.cmake"
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
        COMPONENT ompl)
endif()

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ompl/ompl.conf"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
  COMPONENT ompl)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if (OMPL_VERSIONED_INSTALL)
    # script to create sym links
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh" @ONLY)
    execute_process(COMMAND
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh"
        "omplapp" WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/.symlinks/" DESTINATION .)
    # script to uninstall sym links
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/uninstall_symlinks.sh.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/uninstall_symlinks.sh" @ONLY)
    add_custom_target(uninstall_links COMMAND
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/uninstall_symlinks.sh")
    add_dependencies(uninstall uninstall_links)
endif()

include(CPackSettings)

option(OMPL_REGISTRATION "Enable one-time registration of OMPL" ON)
if (OMPL_REGISTRATION)
    find_file(OMPL_REGISTERED ".registered" PATHS "${CMAKE_CURRENT_SOURCE_DIR}" NO_DEFAULT_PATH)
    if (NOT OMPL_REGISTERED)
        file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.registered" "")
        find_package(Python QUIET)
        if (PYTHON_FOUND)
            execute_process(COMMAND "${PYTHON_EXEC}" "-m" "webbrowser" "http://ompl.kavrakilab.org/register.html"
                OUTPUT_QUIET ERROR_QUIET)
        endif()
    endif()
endif()
