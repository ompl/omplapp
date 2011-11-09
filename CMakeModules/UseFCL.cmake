#find_library(FCL_LIBRARY FCL DOC "Location of FCL collision checking library")
#find_path(FCL_INCLUDE_DIR collision_object.h PATH_SUFFIXES "fcl"
#    DOC "Location of FCL header files")

find_library (ANN_LIBRARY ann DOC "Location of the ANN (approximate nearest neighbor library)")
find_path (ANN_INCLUDE_DIR ANN.h PATH_SUFFIXES "ANN")

if (ANN_LIBRARY AND ANN_INCLUDE_DIR)
message (STATUS "Found ann: ${ANN_LIBRARY}")

# Check for fcl and ccd installation, otherwise download them.
find_library (CCD_LIBRARY ccd DOC "Location of the CCD library (convex collision detection)")
find_path (CCD_INCLUDE_DIR ccd.h PATH_SUFFIXES "ccd")

    ### CCD LIBRARY ###
    if (CCD_LIBRARY AND CCD_INCLUDE_DIR)
        message (STATUS "Found ccd: ${CCD_LIBRARY}")

    else (CCD_LIBRARY AND CCD_INCLUDE_DIR)

        message (STATUS "CCD library not found.  Will download and compile")
        include(ExternalProject)
        # download ccd
        ExternalProject_Add (ccd
            DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
            URL "http://libccd.danfis.cz/files/libccd-1.0.tar.gz"
            CMAKE_ARGS
                "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/ccd-prefix"
                "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
                "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/ccd-prefix/src/ccd-build"
                "-DCMAKE_MODULE_PATH=${CMAKE_SOURCE_DIR}/ompl/CMakeModules"
                "-DCMAKE_VERBOSE_MAKEFILE=ON"
            INSTALL_COMMAND "")

        # use a CMakeLists.txt file to configure build of ccd
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
        message (STATUS "Found fcl: ${FCL_LIBRARY}")

    else (FCL_LIBRARY AND FCL_INCLUDE_DIR)
        message (STATUS "FCL library not found.  Will download and compile")

        include(ExternalProject)
        # download and build FCL
        ExternalProject_Add(fcl
            DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
            SVN_REPOSITORY "https://kforge.ros.org/fcl/fcl_ros/trunk/fcl"
            SVN_REVISION "-r42"
            UPDATE_COMMAND ""
            CMAKE_ARGS
                "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/fcl-prefix"
                "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
                "-DCMAKE_INSTALL_NAME_DIR=${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl-build"
                "-DCMAKE_MODULE_PATH=${CMAKE_SOURCE_DIR}/ompl/CMakeModules"
                "-DANN_INCLUDE_DIR=${CMAKE_BINARY_DIR}/ann/include"
                "-DCCD_INCLUDE_DIR=${CCD_INCLUDE_DIR}"
            INSTALL_COMMAND "")

        # use a CMakeLists.txt file to configure build of FCL
        ExternalProject_Add_Step(fcl addCMakeList
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_SOURCE_DIR}/src/external/CMakeLists-FCL.txt"
                "${CMAKE_BINARY_DIR}/fcl-prefix/src/fcl/CMakeLists.txt"
            DEPENDEES download
            DEPENDERS configure)

        # HACK ALERT:
        # FCL expects ANN headers to be inside a directory called 'ann', not 'ANN'.
        # We cannot simply rename the directory without breaking the ANN code.  
        # Adding a symbolic link isn't likely to be transferable across OS lines, so 
        # lets just copy the directory locally and use that for fcl.
        ExternalProject_Add_Step (fcl copyANNIncs
            COMMAND mkdir -p ${CMAKE_BINARY_DIR}/ann/include/

            COMMAND cp -r 
                ${ANN_INCLUDE_DIR}
                ${CMAKE_BINARY_DIR}/ann/include/ann
            DEPENDERS build)

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
    message (STATUS "FCL Libraries ${FCL_LIBRARIES}")

    #Define USE_FCL so that FCL code is compiled into OMPL.app
    add_definitions (-DUSE_FCL)

else (ANN_LIBRARY AND ANN_INCLUDE_DIR)

message (STATUS "ANN library not found.  Skipping FCL collision checking compilation")

endif (ANN_LIBRARY AND ANN_INCLUDE_DIR)
