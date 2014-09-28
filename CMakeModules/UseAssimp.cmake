find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
    include(FindPkgConfig)
    pkg_check_modules(ASSIMP assimp)
endif()

if(ASSIMP_FOUND AND ASSIMP_LIBRARIES AND ASSIMP_INCLUDE_DIRS)
    if(ASSIMP_LIBRARY_DIRS)
        link_directories(${ASSIMP_LIBRARY_DIRS})
    endif()
    if(${ASSIMP_VERSION} STRGREATER "2.0.0")
        set(OMPL_HAS_ASSIMP3 1)
    else()
        set(OMPL_HAS_ASSIMP2 1)
    endif()
    # if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARIES ASSIMP_INCLUDE_DIRS)
else(ASSIMP_FOUND AND ASSIMP_LIBRARIES AND ASSIMP_INCLUDE_DIRS)

    message (STATUS "Assimp library not found.  Will download and compile.")
    include(ExternalProject)
    # download and build assimp
    ExternalProject_Add(assimp
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        URL "http://sourceforge.net/projects/assimp/files/assimp-3.0/assimp--3.0.1270-source-only.zip"
        URL_MD5 "52aa4cf4e34e6b2a9c5f6c0b3c319af1"
        CMAKE_ARGS
            "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/assimp-prefix"
            "-DCMAKE_BUILD_TYPE=Release" "-DBUILD_ASSIMP_TOOLS=OFF" "-DBUILD_STATIC_LIB=ON"
            "-DCMAKE_CXX_FLAGS=-fPIC" "-DCMAKE_C_FLAGS=-fPIC")

    set(__pkg_config_checked_ASSIMP 99999 CACHE INTERNAL "Assimp found" FORCE)
    set(ASSIMP_FOUND ON CACHE BOOL "Assimp found" FORCE)
    set(ASSIMP_VERSION "3.0.1270" CACHE STRING "Assimp found" FORCE)
    set(OMPL_HAS_ASSIMP3 1)
    set(ASSIMP_LIBRARIES "${CMAKE_BINARY_DIR}/assimp-prefix/lib/${CMAKE_STATIC_LIBRARY_PREFIX}assimp${CMAKE_STATIC_LIBRARY_SUFFIX}")
    # need to add zlib: static assimp lib doesn't pull in zlib dependency
    set(ASSIMP_LIBRARIES "${ASSIMP_LIBRARIES};z" CACHE FILEPATH "Location of 3D asset importer library" FORCE)
    set(ASSIMP_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/assimp-prefix/include")
    if(IS_DIRECTORY "${ASSIMP_INCLUDE_DIRS}")
        set(ASSIMP_INCLUDE_DIRS "${ASSIMP_INCLUDE_DIRS}" CACHE PATH "Location of 3D asset importer header file directory" FORCE)
    endif()
endif(ASSIMP_FOUND AND ASSIMP_LIBRARIES AND ASSIMP_INCLUDE_DIRS)
