find_library(PQP_LIBRARY PQP DOC "Location of PQP proximity query library")
find_path(PQP_INCLUDE_DIR PQP.h PATH_SUFFIXES "PQP"
    DOC "Location of PQP proximity query header files")

if(PQP_LIBRARY AND PQP_INCLUDE_DIR)
    # if already installed, show that library and headers were found
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(pqp DEFAULT_MSG PQP_LIBRARY PQP_INCLUDE_DIR)
else(PQP_LIBRARY AND PQP_INCLUDE_DIR)

    message (STATUS "PQP library not found.  Will download and compile.")
    include(ExternalProject)
    # download and build PQP
    ExternalProject_Add(pqp
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
        URL "http://gamma.cs.unc.edu/software/downloads/SSV/pqp-1.3.tar.gz"
        URL_MD5 "f710e24a62db763d61d08667439d46fd"
        CMAKE_ARGS
            "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/pqp-prefix"
            "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
            "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/pqp-prefix/src/pqp-build"
            "-DCMAKE_MODULE_PATH=${CMAKE_SOURCE_DIR}/ompl/CMakeModules"
        INSTALL_COMMAND "")
    # use a CMakeLists.txt file to configure build of PQP
    ExternalProject_Add_Step(pqp addCMakeList
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/src/external/CMakeLists-PQP.txt"
            "${CMAKE_BINARY_DIR}/pqp-prefix/src/pqp/CMakeLists.txt"
        DEPENDEES download
        DEPENDERS configure)
    # if using Visual Studio, the code is automatically patched
    if(MSVC_IDE)
        ExternalProject_Add_Step(pqp patchForMSVC
            COMMAND ${CMAKE_COMMAND}
                -D "PQP_INCLUDE=${CMAKE_BINARY_DIR}/pqp-prefix/src/pqp/PQP_v1.3/src"
                -P "${CMAKE_SOURCE_DIR}/src/external/patchPQP-MSVC.cmake"
            DEPENDEES download
            DEPENDERS configure)
    endif(MSVC_IDE)

    # set the library and include variables
    set(PQP_LIBRARY "${CMAKE_BINARY_DIR}/pqp-prefix/src/pqp-build/${CMAKE_SHARED_LIBRARY_PREFIX}PQP${CMAKE_STATIC_LIBRARY_SUFFIX}")
    if(EXISTS "${PQP_LIBRARY}")
        set(PQP_LIBRARY "${PQP_LIBRARY}" CACHE FILEPATH "Location of PQP proximity query library" FORCE)
    endif()
    set(PQP_INCLUDE_DIR "${CMAKE_BINARY_DIR}/pqp-prefix/src/pqp/PQP_v1.3/src")
    if(IS_DIRECTORY "${PQP_INCLUDE_DIR}")
        set(PQP_INCLUDE_DIR "${PQP_INCLUDE_DIR}" CACHE PATH "Location of PQP proximity query header files" FORCE)
    endif()
endif(PQP_LIBRARY AND PQP_INCLUDE_DIR)
