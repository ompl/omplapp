include(FindPackageHandleStandardArgs)

find_library (ANN_LIBRARY NAMES ann ANN DOC "Location of the ANN (approximate nearest neighbor library)")
find_path (ANN_INCLUDE_DIR ANN)

if (ANN_LIBRARY AND ANN_INCLUDE_DIR)
    find_package_handle_standard_args(ann DEFAULT_MSG ANN_LIBRARY ANN_INCLUDE_DIR)

    # Check for FCL and CCD installation, otherwise download them.
    # ANN is required for FCL, so don't download anything unless
    # ANN is installed.

    ### CCD LIBRARY ###
    find_library (CCD_LIBRARY ccd DOC "Location of the CCD library (convex collision detection)")
    find_path (CCD_INCLUDE_DIR ccd)
    if (CCD_LIBRARY AND CCD_INCLUDE_DIR)
        find_package_handle_standard_args(ccd DEFAULT_MSG CCD_LIBRARY CCD_INCLUDE_DIR)
    else (CCD_LIBRARY AND CCD_INCLUDE_DIR)

        message (STATUS "CCD library not found.  Will download and compile.")
        include(ExternalProject)
        # download ccd
        ExternalProject_Add (ccd
            DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
            URL "http://libccd.danfis.cz/files/libccd-1.1.tar.gz"
            URL_MD5 "9c8d3606486188f6e09fe4836711537c"
            CMAKE_ARGS
                "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/ccd-prefix"
                "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
                "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/ccd-prefix/src/ccd-build"
                "-DCMAKE_VERBOSE_MAKEFILE=ON" "-DCCD_DOUBLE=1"
            INSTALL_COMMAND "")
            
        # use a CMakeLists.txt file to configure build of libccd
        ExternalProject_Add_Step(ccd addCMakeList
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_SOURCE_DIR}/src/external/CMakeLists-CCD.txt"
                "${CMAKE_BINARY_DIR}/ccd-prefix/src/ccd/CMakeLists.txt"
            DEPENDEES download
            DEPENDERS configure)

        # Set the CCD_LIBRARY Variable
        set(CCD_LIBRARY "${CMAKE_BINARY_DIR}/ccd-prefix/src/ccd-build/${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX}")
        if(EXISTS "${CCD_LIBRARY}")
            set(CCD_LIBRARY "${CCD_LIBRARY}" CACHE FILEPATH "Location of convex collision detection library" FORCE)
        endif()

        # Set the CCD_INCLUDE_DIR Variable
        set(CCD_INCLUDE_DIR "${CMAKE_BINARY_DIR}/ccd-prefix/src/ccd/src")
        if(IS_DIRECTORY "${CCD_INCLUDE_DIR}")
            set(CCD_INCLUDE_DIR "${CCD_INCLUDE_DIR}" CACHE PATH "Location of convex collision detection header files" FORCE)
        endif()
    endif (CCD_LIBRARY AND CCD_INCLUDE_DIR)

    ### FCL LIBRARY ###
    find_library (FCL_LIBRARY FCL DOC "Location of FCL collision checking library")
    find_path (FCL_INCLUDE_DIR collision_object.h PATH_SUFFIXES "fcl" DOC "Location of FCL header files")

    if (FCL_LIBRARY AND FCL_INCLUDE_DIR)
        find_package_handle_standard_args(fcl DEFAULT_MSG FCL_LIBRARY FCL_INCLUDE_DIR)
    else (FCL_LIBRARY AND FCL_INCLUDE_DIR)
        message (STATUS "FCL library not found.  Will download and compile.")

        include(ExternalProject)
        # download and build FCL
        ExternalProject_Add(fcl
            DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
            SVN_REPOSITORY "https://kforge.ros.org/fcl/fcl_ros/trunk/fcl"
            SVN_REVISION "-r71"
            SVN_TRUST_CERT 1
            UPDATE_COMMAND ""
            CMAKE_ARGS
                "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/fcl-prefix"
                "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
                "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl-build"
                "-DCMAKE_MODULE_PATH=${CMAKE_SOURCE_DIR}/ompl/CMakeModules"
                "-DANN_INCLUDE_DIR=${ANN_INCLUDE_DIR}"
                "-DCCD_INCLUDE_DIR=${CCD_INCLUDE_DIR}"
            INSTALL_COMMAND "")

        # use a CMakeLists.txt file to configure build of FCL
        ExternalProject_Add_Step(fcl addCMakeList
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_SOURCE_DIR}/src/external/CMakeLists-FCL.txt"
                "${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl/CMakeLists.txt"
            DEPENDEES download
            DEPENDERS configure)

        # Make sure ccd exists before building fcl.
        add_dependencies(fcl ccd)

        set(FCL_LIBRARY "${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl-build/${CMAKE_SHARED_LIBRARY_PREFIX}fcl${CMAKE_STATIC_LIBRARY_SUFFIX}")
        if(EXISTS "${FCL_LIBRARY}")
            set(FCL_LIBRARY "${FCL_LIBRARY}" CACHE FILEPATH "Location of FCL collision checking library" FORCE)
        endif()
        set(FCL_INCLUDE_DIR "${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl/include")
        if(IS_DIRECTORY "${FCL_INCLUDE_DIR}")
            set(FCL_INCLUDE_DIR "${FCL_INCLUDE_DIR}" CACHE PATH "Location of FCL collision checker header files" FORCE)
        endif()
    endif (FCL_LIBRARY AND FCL_INCLUDE_DIR)

    set(FCL_LIBRARIES ${ANN_LIBRARY} ${CCD_LIBRARY} ${FCL_LIBRARY})

endif (ANN_LIBRARY AND ANN_INCLUDE_DIR)
