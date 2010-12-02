find_package(Drawstuff QUIET)

if( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
    # if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(drawstuff DEFAULT_MSG DRAWSTUFF_LIBRARY DRAWSTUFF_INCLUDE_DIR)
else( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
    include(ExternalProject)
    # download and build PQP
    ExternalProject_Add(drawstuff
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        SVN_REPOSITORY "https://opende.svn.sourceforge.net/svnroot/opende/trunk"
        SVN_REVISION "-r1770"
        CONFIGURE_COMMAND "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/configure" 
            "--enable-double-precision" "--with-pic"
        UPDATE_COMMAND ""
        BUILD_IN_SOURCE 1
        INSTALL_COMMAND "")
    # use a CMakeLists.txt file to configure build of PQP
    ExternalProject_Add_Step(drawstuff autogen
        COMMAND "./autogen.sh"
        DEPENDEES download
        DEPENDERS configure
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff")
    
    # set the library and include variables
    set(DRAWSTUFF_LIBRARY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/drawstuff/src/.libs/libdrawstuff.a" 
        CACHE FILEPATH "Location of ODE's drawstuff library" FORCE)
    set(DRAWSTUFF_INCLUDE_DIR "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/include"
        CACHE PATH "Location of ODE's drawstuff header files" FORCE)
    set(DRAWSTUFF_FOUND TRUE)
    
    # since we will download and build ODE, we might as well use the 
    # ODE library and header files *unless* another version of ODE has
    # already been found.
    if(NOT ODE_FOUND)
        set(ODE_LIBRARY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/ode/src/.libs/libode.a" 
            CACHE FILEPATH "Location of ODE library" FORCE)
        set(ODE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/include"
            CACHE PATH "Location of ODE header files" FORCE)
        set(ODE_DEFINITIONS "-DdDOUBLE")
        set(ODE_VERSION "svn-r1770")
        set(ODE_FOUND TRUE)        
    endif(NOT ODE_FOUND)
endif( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
