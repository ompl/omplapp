find_library(ASSIMP_LIBRARY assimp DOC "Location of 3D asset importer library")
find_path(ASSIMP_INCLUDE_DIR assimp.h PATH_SUFFIXES assimp 
   DOC "Location of 3D asset importer header file directory")

if(ASSIMP_LIBRARY AND ASSIMP_INCLUDE_DIR)
    # if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARY ASSIMP_INCLUDE_DIR)
else(ASSIMP_LIBRARY AND ASSIMP_INCLUDE_DIR)
    include(ExternalProject)
    # download and build assimp
    ExternalProject_Add(assimp
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        URL "http://softlayer.dl.sourceforge.net/assimp/assimp--2.0.858-sdk.zip"
        URL_MD5 "01b92c012ab52e8947dea1bda03fad9f"
        CMAKE_ARGS 
            "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/assimp-prefix"
            "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
            "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/assimp-prefix/lib")

    # set the library and include variables
    set(ASSIMP_LIBRARY "${CMAKE_BINARY_DIR}/assimp-prefix/lib/${CMAKE_SHARED_LIBRARY_PREFIX}assimp${CMAKE_SHARED_LIBRARY_SUFFIX}"
        CACHE FILEPATH "Location of 3D asset importer library" FORCE)
    set(ASSIMP_INCLUDE_DIR "${CMAKE_BINARY_DIR}/assimp-prefix/include/assimp"
        CACHE PATH "Location of 3D asset importer header file directory" FORCE)
endif(ASSIMP_LIBRARY AND ASSIMP_INCLUDE_DIR)
