cmake_minimum_required(VERSION 2.8)
if(${CMAKE_VERSION} VERSION_GREATER 2.8.3)
    cmake_policy(SET CMP0017 NEW)
endif()
project(omplapp CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
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
set(OMPL_DOC_INSTALL_DIR "share/ompl${OMPL_INSTALL_SUFFIX}/doc"
    CACHE STRING "Relative path to directory where documentation will be installed")
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

find_package(Boost COMPONENTS date_time thread program_options serialization filesystem system unit_test_framework REQUIRED)
find_package(Boost COMPONENTS python)

find_package(Boost COMPONENTS odeint QUIET)
if(NOT Boost_ODEINT_FOUND)
    # Include bundled version of boost::odeint if it isn't installed natively
    set(ODEINT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src/external")
    include_directories("${ODEINT_INCLUDE_DIR}")
endif()
# needed to compile ODESolver demo. ODEsolver is *required* to compile ompl_app.
set(OMPL_ODESOLVER ON)

find_package(OpenGL)
# This includes our own FindPython.cmake in ompl/CMakeModules. It defines,
# among other things, the find_python_module() function used below.
find_package(Python)
find_python_module(PyQt4 QUIET)
find_python_module(PySide QUIET)
if(NOT PY_PYQT4 AND NOT PY_PYSIDE)
    message(WARNING "Either PyQt4 or PySide needs to be installed to use the GUI.")
endif()
find_python_module(OpenGL)
if (NOT OPENGL_FOUND OR NOT PY_OPENGL)
    message(WARNING "Both OpenGL and the Python OpenGL module need to be installed to use the GUI")
endif()
find_package(OpenDE QUIET)
find_package(Drawstuff QUIET)
find_package(PQP QUIET)
# The ExternalProject.cmake that comes with CMake 2.8.[01] is broken
# and may not even exist in earlier versions of CMake. We need this
# module to download and build FCL and Assimp (if they aren't
# already installed).
if(${CMAKE_VERSION} VERSION_LESS 2.8.4 AND
    NOT EXISTS "${CMAKE_SOURCE_DIR}/CMakeModules/ExternalProject.cmake")
    file(DOWNLOAD
            "http://cmake.org/gitweb?p=cmake.git;a=blob_plain;f=Modules/ExternalProject.cmake;hb=HEAD"
            "${CMAKE_SOURCE_DIR}/CMakeModules/ExternalProject.cmake")
endif()
include(UseFCL)
include(UseAssimp)

enable_testing()

include_directories(
    "${OMPLAPP_INCLUDE_DIR}"
    "${OMPL_INCLUDE_DIR}"
    "${FCL_INCLUDE_DIRS}"
    "${CCD_INCLUDE_DIRS}"
    "${Boost_INCLUDE_DIR}"
    "${ASSIMP_INCLUDE_DIRS}")

set(OMPLAPP_MODULE_LIBRARIES
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARY}
    ${FCL_LIBRARIES})
set(OMPLAPP_LIBRARIES
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SERIALIZATION}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES})
link_directories(${CCD_LIBDIR} ${FCL_LIBDIR})

if(OPENGL_INCLUDE_DIR)
    include_directories("${OPENGL_INCLUDE_DIR}")
endif()

set(OMPL_EXTENSION_OPENDE ${OPENDE_FOUND})
if (OPENDE_FOUND)
    add_definitions(${OPENDE_DEFINITIONS})
    include_directories("${OPENDE_INCLUDE_DIR}")
endif()

option(OMPLAPP_PQP "Enable support for the PQP collision checking library" ON)
if(OMPLAPP_PQP AND PQP_FOUND)
    include_directories("${PQP_INCLUDE_DIR}")
    set(OMPLAPP_MODULE_LIBRARIES ${OMPLAPP_MODULE_LIBRARIES} ${PQP_LIBRARY})
    set(OMPLAPP_LIBRARIES ${OMPLAPP_LIBRARIES} ${PQP_LIBRARY})
endif()


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
install(DIRECTORY resources DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}"
    PATTERN ".svn" EXCLUDE
    PATTERN ".DS_Store" EXCLUDE)

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


if (NOT MSVC)
    set(PKG_NAME "ompl")
    set(PKG_DESC "The Open Motion Planning Library")
    set(PKG_OMPL_LIBS "-lompl -lompl_app")
    set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/ompl.pc")
    configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
    install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/
        COMPONENT pkgconfig RENAME "ompl${OMPL_INSTALL_SUFFIX}.pc")

    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ompl/doc/dox/FindOMPL.cmake"
        DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}"
        RENAME ompl-config.cmake)
    if (NOT ${CMAKE_VERSION} VERSION_LESS 2.8.5)
        include(WriteBasicConfigVersionFile)
        write_basic_config_version_file(
            ${CMAKE_CURRENT_BINARY_DIR}/ompl-config-version.cmake
            VERSION ${OMPL_VERSION} COMPATIBILITY SameMajorVersion)
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ompl-config-version.cmake
            DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}")
    endif()
endif()

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

if (OMPL_VERSIONED_INSTALL)
    # script to create sym links
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh" @ONLY)
    execute_process(COMMAND
      "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/create_symlinks.sh"
      "omplapp" WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/.symlinks/ DESTINATION .)
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
