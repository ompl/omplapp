cmake_minimum_required(VERSION 3.12)
project(omplapp VERSION 1.6.0 LANGUAGES CXX)
set(OMPL_ABI_VERSION 17)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(GNUInstallDirs)
include(FeatureSummary)
# include(CompilerSettings)
# include(OMPLUtils)

option(USE_BUNDLED_OMPL "Use OMPL from subdirectory instead of system install" OFF)
if(NOT USE_BUNDLED_OMPL)
    find_package(ompl CONFIG QUIET)
    if(ompl_FOUND)
        message(STATUS "Using installed OMPL")
    else()
        message(WARNING "OMPL not found. Falling back to bundled OMPL")
        set(USE_BUNDLED_OMPL ON CACHE BOOL "Use bundled OMPL" FORCE)
    endif()
endif()

# Only include custom modules if OMPL is being built as subdirectory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/CompilerSettings.cmake")
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
    include(CompilerSettings)
    include(OMPLUtils)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(OMPLAPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")
set(OMPLAPP_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")

if(NOT USE_BUNDLED_OMPL)
    # OMPL is installed, we use its targets
    message(STATUS "Using installed OMPL")
else()
    # OMPL is bundled, we include manually
    # include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ompl/src")
    set(OMPL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src;${CMAKE_CURRENT_BINARY_DIR}/ompl/src")
    set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules"
        CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
endif()

set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
        CACHE STRING "Relative path to directory where demos will be installed")

# set(OMPL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src;${CMAKE_CURRENT_BINARY_DIR}/ompl/src")
# set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
#     CACHE STRING "Relative path to directory where demos will be installed")
# set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules"
#     CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
# Ensure dynamic linking with boost unit_test_framework
add_definitions(-DBOOST_TEST_DYN_LINK)
# Avoid valgrind error due to overflow error, cf. https://github.com/ompl/ompl/issues/664
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

set_package_properties(Boost PROPERTIES
    URL "http://boost.org"
    PURPOSE "Used throughout OMPL for data serialization, graphs, etc.")
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.58 REQUIRED COMPONENTS serialization filesystem system program_options)

# on macOS we need to check whether to use libc++ or libstdc++ with clang++
if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
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

set_package_properties(OpenGL PROPERTIES
    URL "http://opengl.org"
    PURPOSE "Used to render graphics in ompl_app.")
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL)
set_package_properties(Python PROPERTIES
    URL "http://python.org"
    PURPOSE "Used for python bindings.")
# This includes our own FindPython.cmake in ompl/CMakeModules. It defines,
# among other things, the find_python_module() function used below.
find_package(Python)
# find_boost_python()

if(USE_BUNDLED_OMPL AND NOT TARGET docker)
    include(OMPLUtils)
    find_boost_python()
    find_python_module(PyQt5 QUIET)
else()
    message(STATUS "Skipping internal OMPL Python utilities when using external OMPL install")
endif()

if(PYTHON_FOUND)
    set_package_properties(pypy PROPERTIES
        URL "http://pypy.org"
        PURPOSE "Used to speed up the generation of python bindings.")
    find_package(pypy QUIET)
endif()

if(USE_BUNDLED_OMPL)
    find_python_module(PyQt5 QUIET)
    if(NOT PY_PYQT5)
        find_python_module(PySide2 QUIET)
        if(NOT PY_PYSIDE2)
            find_python_module(PyQt4 QUIET)
            if(NOT PY_PYQT4)
                find_python_module(PySide QUIET)
                if(NOT PY_PYSIDE)
                    message(WARNING "Either PyQt5, PySide2, PyQt4 or PySide needs to be installed to use the GUI.")
                endif()
            endif()
        endif()
    endif()

    find_python_module(OpenGL)
    if(NOT OPENGL_FOUND OR NOT PY_OPENGL)
        message(WARNING "Both OpenGL and the Python OpenGL module need to be installed to use the GUI")
    endif()
endif()

# find_python_module(PyQt5 QUIET)
# if(NOT PY_PYQT5)
#     find_python_module(PySide2 QUIET)
#     if(NOT PY_PYSIDE2)
#         find_python_module(PyQt4 QUIET)
#         if(NOT PY_PYQT4)
#             find_python_module(PySide QUIET)
#             if (NOT PY_PYSIDE)
#                 message(WARNING "Either PyQt5, PySide2, PyQt4 or PySide needs to be installed to use the GUI.")
#             endif()
#         endif()
#     endif()
# endif()
# find_python_module(OpenGL)
# if (NOT OPENGL_FOUND OR NOT PY_OPENGL)
#     message(WARNING "Both OpenGL and the Python OpenGL module need to be installed to use the GUI")
# endif()
set_package_properties(Eigen3 PROPERTIES
    URL "http://eigen.tuxfamily.org"
    PURPOSE "A linear algebra library used throughout OMPL.")
find_package(Eigen3 REQUIRED)
set_package_properties(Triangle PROPERTIES
    URL "http://www.cs.cmu.edu/~quake/triangle.html"
    PURPOSE "Used to create triangular decompositions of polygonal 2D environments.")
find_package(Triangle QUIET)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})

set_package_properties(flann PROPERTIES
    URL "https://github.com/mariusmuja/flann"
    PURPOSE "If detetected, FLANN can be used for nearest neighbor queries by OMPL.")
find_package(flann CONFIG 1.9.2 QUIET)
if (flann_FOUND)
    set(OMPL_HAVE_FLANN 1)
endif()

set_package_properties(spot PROPERTIES
    URL "http://spot.lrde.epita.fr"
    PURPOSE "Used for constructing finite automata from LTL formulae.")
find_package(spot)
if (spot_FOUND)
    set(OMPL_HAVE_SPOT 1)
endif()

set_package_properties(assimp PROPERTIES
    URL "http://assimp.org"
    PURPOSE "Used in ompl_app for reading meshes representing robots and environments.")
find_package(assimp REQUIRED)

set_package_properties(fcl PROPERTIES
    URL "https://github.com/flexible-collision-library/fcl"
    PURPOSE "The default collision checking library.")
find_package(fcl REQUIRED)

set_package_properties(Threads PROPERTIES
    URL "https://en.wikipedia.org/wiki/POSIX_Threads"
    PURPOSE "Pthreads is sometimes needed, depending on OS / compiler.")
find_package(Threads QUIET)

set_package_properties(Doxygen PROPERTIES
    URL "http://doxygen.org"
    PURPOSE "Used to create the OMPL documentation (i.e., http://ompl.kavrakilab.org).")
find_package(Doxygen)

enable_testing()

# Add support in Boost::Python for std::shared_ptr
# This is a hack that replaces boost::shared_ptr related code with std::shared_ptr.
# Proper support for std::shared_ptr was added in Boost 1.63.
if (NOT Boost_VERSION_STRING)
    if (Boost_VERSION_MAJOR)
        set(Boost_VERSION_STRING
            "${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR}.${Boost_VERSION_PATCH}")
    else()
        set(Boost_VERSION_STRING
            "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_PATCH_VERSION}")
    endif()
endif()
if(Boost_VERSION_STRING VERSION_LESS "1.63.0")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ompl/src/external")
endif()

# include_directories(
#     "${OMPLAPP_INCLUDE_DIRS}"
#     "${OMPL_INCLUDE_DIRS}")

if(OMPL_INCLUDE_DIRS)
    include_directories("${OMPL_INCLUDE_DIRS}")
endif()

include_directories("${OMPLAPP_INCLUDE_DIRS}")



set(OMPLAPP_MODULE_LIBRARIES
    ${OPENGL_LIBRARIES}
    assimp::assimp
    fcl::fcl)
set(OMPLAPP_LIBRARIES
    ${OPENGL_LIBRARIES}
    assimp::assimp
    fcl)

if (OPENGL_INCLUDE_DIR)
    include_directories("${OPENGL_INCLUDE_DIR}")
endif()

# Numpy is used to convert Eigen matrices/vectors to numpy arrays
# if(PYTHON_FOUND AND NOT Boost_VERSION_STRING VERSION_LESS "1.63.0")
#     find_python_module(numpy)
#     if (PY_NUMPY)
#         find_boost_numpy()
#         if(Boost_NUMPY_LIBRARY)
#             set(OMPL_HAVE_NUMPY 1)
#             include_directories("${PY_NUMPY}/core/include")
#         endif()
#     endif()
# endif()

if(USE_BUNDLED_OMPL AND PYTHON_FOUND AND NOT Boost_VERSION_STRING VERSION_LESS "1.63.0")
    find_python_module(numpy)
    if (PY_NUMPY)
        find_boost_numpy()
        if(Boost_NUMPY_LIBRARY)
            set(OMPL_HAVE_NUMPY 1)
            include_directories("${PY_NUMPY}/core/include")
        endif()
    endif()
endif()


set_package_properties(PQP PROPERTIES
    URL "http://gamma.cs.unc.edu/SSV/"
    PURPOSE "Used as an alternative, additional collision checking library (the default is FCL).")
find_package(PQP QUIET)
if(PQP_FOUND)
    include_directories("${PQP_INCLUDE_DIR}")
    set(OMPLAPP_MODULE_LIBRARIES ${OMPLAPP_MODULE_LIBRARIES} ${PQP_LIBRARY})
    set(OMPLAPP_LIBRARIES ${OMPLAPP_LIBRARIES} ${PQP_LIBRARY})
endif()

# find_package(ompl CONFIG QUIET)

if(NOT ompl_FOUND)
    message(WARNING "Installed OMPL not found. Falling back to building bundled OMPL.")

    set(OMPL_BUILD_PYBINDINGS OFF CACHE BOOL "Disable Python bindings in OMPL")
    add_subdirectory(ompl/doc)
    add_subdirectory(ompl/src)
    add_subdirectory(ompl/py-bindings)
    add_subdirectory(ompl/tests)
    add_subdirectory(ompl/demos)
    add_subdirectory(ompl/scripts)
    set(USE_BUNDLED_OMPL TRUE)
else()
    set(USE_BUNDLED_OMPL FALSE)
endif()

if(NOT USE_BUNDLED_OMPL)
    function(install_python)
        # No-op macro when using installed OMPL
    endfunction()
endif()

if(NOT USE_BUNDLED_OMPL)
    function(target_link_flags)
        # noop fallback for external OMPL installs
    endfunction()
endif()


# add_subdirectory(ompl/doc)
# add_subdirectory(ompl/src)
# add_subdirectory(ompl/py-bindings)
# add_subdirectory(ompl/tests)
# add_subdirectory(ompl/demos)
# add_subdirectory(ompl/scripts)
add_subdirectory(gui)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(demos)
add_subdirectory(benchmark)
# add_subdirectory(py-bindings)

if(USE_BUNDLED_OMPL)
  add_subdirectory(py-bindings)
endif()


install(DIRECTORY resources
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl"
    COMPONENT omplapp
    PATTERN ".DS_Store" EXCLUDE)

if(OMPL_BUILD_PYBINDINGS)
    install(DIRECTORY webapp
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl"
        COMPONENT omplapp
        PATTERN ".DS_Store" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "omplweb.py" EXCLUDE)
    install_python(PROGRAMS webapp/omplweb.py
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl/webapp"
        COMPONENT omplapp)
    find_python_module(flask)
    find_python_module(celery)
    find_program(CELERY celery)
    if(PY_FLASK AND PY_CELERY AND CELERY)
        install(PROGRAMS webapp/ompl_webapp
            DESTINATION bin
            RENAME ompl_webapp)
    endif()
endif()

# if(OPENGL_FOUND AND NOT MSVC)
if(OPENGL_FOUND AND NOT MSVC AND USE_BUNDLED_OMPL)
    target_link_flags(ompl ompl_app_base ompl_app)
    set(PKG_NAME "ompl")
    set(PKG_DESC "The Open Motion Planning Library")
    set(PKG_EXTERNAL_DEPS "${ompl_PKG_DEPS}")
    set(PKG_OMPL_LIBS "-lompl -lompl_app_base -lompl_app ${ompl_LINK_FLAGS}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/ompl.pc.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
        COMPONENT ompl)
endif()

# include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
# configure_package_config_file(ompl/omplConfig.cmake.in
#     ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
#     INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
#     PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
#     NO_CHECK_REQUIRED_COMPONENTS_MACRO)
# write_basic_package_version_file(
#     ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY SameMajorVersion)
# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
#     ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
#     COMPONENT ompl)
# install(TARGETS ompl
#     EXPORT omplExport
#     DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     COMPONENT ompl)

install(TARGETS ompl_app_base
    EXPORT omplExport
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT omplapp)
if(TARGET ompl_app)
    install(TARGETS ompl_app
        EXPORT omplExport
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT omplapp)
endif()
# install(EXPORT omplExport
#     NAMESPACE ompl::
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake)
# export(EXPORT omplExport
#     FILE "${CMAKE_CURRENT_BINARY_DIR}/omplExport.cmake")

# script to install ompl on Ubuntu
# configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/install-ompl-ubuntu.sh.in"
#   "${CMAKE_CURRENT_BINARY_DIR}/install-ompl-ubuntu.sh" @ONLY)

# uninstall target
# configure_file(
#     "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/cmake_uninstall.cmake.in"
#     "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#     IMMEDIATE @ONLY)
# add_custom_target(uninstall
#     COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

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

set_package_properties(PkgConfig PROPERTIES
    URL "https://www.freedesktop.org/wiki/Software/pkg-config/"
    PURPOSE "Used to find (compilation flags for) dependencies.")
set_package_properties(castxml PROPERTIES
    URL "https://github.com/CastXML/CastXML"
    PURPOSE "Used to generate Python bindings.")
feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
# additional feature info: show which Python modules were found and weren't found
get_property(PY_MODULES_FOUND GLOBAL PROPERTY PY_MODULES_FOUND)
if(PY_MODULES_FOUND)
    list(REMOVE_DUPLICATES PY_MODULES_FOUND)
    string(REPLACE ";" " " PY_MODULES_FOUND_STR "${PY_MODULES_FOUND}")
    message(STATUS "The following Python modules were found:\n\n * ${PY_MODULES_FOUND_STR}\n")
endif()
get_property(PY_MODULES_NOTFOUND GLOBAL PROPERTY PY_MODULES_NOTFOUND)
if(PY_MODULES_NOTFOUND)
    list(REMOVE_DUPLICATES PY_MODULES_NOTFOUND)
    string(REPLACE ";" " " PY_MODULES_NOTFOUND_STR "${PY_MODULES_NOTFOUND}")
    message(STATUS "The following Python modules were NOT found:\n\n * ${PY_MODULES_NOTFOUND_STR}\n")
endif()

# Create targets for building docker images
# See ompl/CMakeModules/OMPLUtils.cmake and ompl/scripts/docker for details
# add_docker_target(ompl ompl)
# add_docker_target(plannerarena ompl/scripts)
# add_docker_target(omplapp)
if(USE_BUNDLED_OMPL)
    # Install script for Ubuntu
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/install-ompl-ubuntu.sh.in"
      "${CMAKE_CURRENT_BINARY_DIR}/install-ompl-ubuntu.sh" @ONLY)

    # Uninstall target
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

    # .pc file for pkg-config
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/ompl.pc.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
        COMPONENT ompl)

    # CMake config files
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        ompl/omplConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
        PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
        COMPONENT ompl)

    install(TARGETS ompl EXPORT omplExport DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ompl)
    install(EXPORT omplExport
        NAMESPACE ompl::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake)
    export(EXPORT omplExport
        FILE "${CMAKE_CURRENT_BINARY_DIR}/omplExport.cmake")

    # Docker targets
    add_docker_target(ompl ompl)
    add_docker_target(plannerarena ompl/scripts)
    add_docker_target(omplapp)
endif()
