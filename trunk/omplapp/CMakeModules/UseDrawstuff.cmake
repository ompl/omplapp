find_package(Drawstuff)

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
        URL_MD5 "f710e24a62db763d61d08667439d46fd"
        CONFIGURE_COMMAND "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/configure" 
            "--enable-double-precision" "--with-pic"
        INSTALL_COMMAND "")
    # use a CMakeLists.txt file to configure build of PQP
    ExternalProject_Add_Step(drawstuff autogen
        COMMAND "./autogen.sh"
        DEPENDEES download
        DEPENDERS configure
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff")
    
    # set the library and include variables
    set(DRAWSTUFF_LIBRARY "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff-build/drawstuff/src/.libs/libdrawstuff.a" 
        CACHE FILEPATH "Location of ODE's drawstuff library" FORCE)
    set(DRAWSTUFF_INCLUDE_DIR "${CMAKE_BINARY_DIR}/drawstuff-prefix/src/drawstuff/include"
        CACHE PATH "Location of ODE's drawstuff header files" FORCE)
    set(DRAWSTUFF_FOUND TRUE)
endif( ( DRAWSTUFF_LIBRARY AND DRAWSTUFF_INCLUDE_DIR ) OR MSVC_IDE)
