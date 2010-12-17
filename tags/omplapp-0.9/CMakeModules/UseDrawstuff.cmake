find_package(Drawstuff QUIET)

if( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
    # if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(drawstuff DEFAULT_MSG DRAWSTUFF_LIBRARY DRAWSTUFF_INCLUDE_DIR)
else( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
    include(ExternalProject)
    # download and build drawstuff (and ODE)
    # we need the svn version of ODE to get a working version of libdrawstuff on OS X
    ExternalProject_Add(drawstuff
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        SVN_REPOSITORY "https://opende.svn.sourceforge.net/svnroot/opende/trunk"
        SVN_REVISION "-r1770"
        CONFIGURE_COMMAND "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/configure" 
            "--enable-double-precision" "--with-pic"
        UPDATE_COMMAND ""
        BUILD_IN_SOURCE 1
        INSTALL_COMMAND "")
    # run autogen.sh to create the configure script
    ExternalProject_Add_Step(drawstuff autogen
        COMMAND "./autogen.sh"
        DEPENDEES download
        DEPENDERS configure
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff")
    
    # set the library and include variables
    set(DRAWSTUFF_LIBRARY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/drawstuff/src/.libs/libdrawstuff.a")
    if(EXISTS "${DRAWSTUFF_LIBRARY}")
        set(DRAWSTUFF_LIBRARY "${DRAWSTUFF_LIBRARY}" CACHE FILEPATH "Location of ODE's drawstuff library" FORCE)
    endif()
    set(DRAWSTUFF_INCLUDE_DIR "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/include")
    if(IS_DIRECTORY "${DRAWSTUFF_INCLUDE_DIR}")
        set(DRAWSTUFF_INCLUDE_DIR "${DRAWSTUFF_INCLUDE_DIR}" CACHE PATH "Location of ODE's drawstuff header files" FORCE)
    endif()
    set(DRAWSTUFF_FOUND TRUE)
    
    # since we will download and build ODE, we might as well use the 
    # ODE library and header files *unless* another version of ODE has
    # already been found.
    if(NOT ODE_FOUND)
        set(ODE_LIBRARY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/ode/src/.libs/libode.a")
        if(EXISTS "${ODE_LIBRARY}")
            set(ODE_LIBRARY "${ODE_LIBRARY}" CACHE FILEPATH "Location of ODE library" FORCE)
        endif()
        set(ODE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/include")
        if(IS_DIRECTORY "${ODE_INCLUDE_DIR}")
            set(ODE_INCLUDE_DIR "${ODE_INCLUDE_DIR}" CACHE PATH "Location of ODE header files" FORCE)
        endif()
        set(ODE_DEFINITIONS "-DdDOUBLE" CACHE STRING "ODE Compile flags" FORCE)
        set(ODE_VERSION "svn-r1770" CACHE STRING "ODE version" FORCE)
        set(ODE_FOUND TRUE CACHE BOOL "Whether ODE was found" FORCE)
    endif(NOT ODE_FOUND)
endif( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
