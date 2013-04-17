include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(CCD ccd=1.4)
    pkg_check_modules(FCL fcl>=0.2.7)
endif()

# Check for FCL and CCD installation, otherwise download them.

### CCD LIBRARY ###
if(NOT CCD_FOUND)
    message (STATUS "CCD library not found.  Will download and compile.")
    include(ExternalProject)
    # download and build ccd
    ExternalProject_Add (ccd
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        URL "http://libccd.danfis.cz/files/libccd-1.4.tar.gz"
        URL_MD5 "684a9f2f44567a12a30af383de992a89"
        CMAKE_ARGS
            "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/ccd-prefix"
            "-DCMAKE_BUILD_TYPE=Release" "-DCCD_DOUBLE=1" "-DCMAKE_C_FLAGS=-fPIC")

    # Set the CCD_LIBRARIES Variable
    set(CCD_LIBDIR "${CMAKE_BINARY_DIR}/ccd-prefix/lib")
    set(CCD_LIBRARIES "${CCD_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}ccd${CMAKE_STATIC_LIBRARY_SUFFIX}")
    if(EXISTS "${CCD_LIBRARIES}")
        set(CCD_LIBRARIES "${CCD_LIBRARIES}" CACHE FILEPATH "Location of convex collision detection library" FORCE)
    endif()

    # Set the CCD_INCLUDE_DIR Variable
    set(CCD_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/ccd-prefix/include")
    if(IS_DIRECTORY "${CCD_INCLUDE_DIRS}")
        set(CCD_INCLUDE_DIRS "${CCD_INCLUDE_DIRS}" CACHE PATH "Location of convex collision detection header files" FORCE)
    endif()
endif(NOT CCD_FOUND)

### FCL LIBRARY ###
if(NOT FCL_FOUND)
    message (STATUS "FCL library not found.  Will download and compile.")
    include(ExternalProject)

    # download and build FCL
    ExternalProject_Add(fcl
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        URL "http://downloads.sourceforge.net/project/ompl/dependencies/fcl-0.2.7.zip"
        URL_MD5 "c9427516b5fb08afa82bc69d15f57787"
        CMAKE_COMMAND env
        CMAKE_ARGS
            "PKG_CONFIG_PATH=${CCD_LIBDIR}/pkgconfig"
            "${CMAKE_COMMAND}"
            "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/fcl-prefix"
            "-DCMAKE_BUILD_TYPE=Release"
            "-DCMAKE_CXX_FLAGS='-fPIC -I${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl/include'"
            "-DFCL_STATIC_LIBRARY=ON"
            "-DCCD_INCLUDE_DIRS=${CCD_INCLUDE_DIRS}"
            "-DCCD_LIBRARY_DIRS=${CCD_LIBDIR}")

    # Make sure ccd exists before building fcl.
    add_dependencies(fcl ccd)

    set(FCL_LIBDIR "${CMAKE_BINARY_DIR}/fcl-prefix/lib")
    set(FCL_LIBRARIES "${FCL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}fcl${CMAKE_STATIC_LIBRARY_SUFFIX}")
    if(EXISTS "${FCL_LIBRARIES}")
        set(FCL_LIBRARIES "${FCL_LIBRARIES}" CACHE FILEPATH "Location of FCL collision checking library" FORCE)
    endif()
    set(FCL_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/fcl-prefix/include")
    if(IS_DIRECTORY "${FCL_INCLUDE_DIRS}")
        set(FCL_INCLUDE_DIRS "${FCL_INCLUDE_DIRS}" CACHE PATH "Location of FCL collision checker header files" FORCE)
    endif()
endif(NOT FCL_FOUND)

# Link order is very important here.  If FCL isn't linked first, over-zealous
# optimization may remove needed symbols from CCD in subsequent links.
set(FCL_LIBRARIES ${FCL_LIBRARIES} ${CCD_LIBRARIES})
