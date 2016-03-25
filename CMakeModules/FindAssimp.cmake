include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(ASSIMP assimp)
    if(ASSIMP_VERSION)
        if(ASSIMP_VERSION STRGREATER "2.0.0")
            set(OMPL_HAS_ASSIMP3 1)
        else()
            set(OMPL_HAS_ASSIMP2 1)
        endif()
    endif()
endif()

# if pkf-config is not installed and user has manually specified
# ASSIMP_LIBRARIES and ASSIMP_INCLUDE_DIRS, then assume it's Assimp3
if(NOT OMPL_HAS_ASSIMP2 and NOT OMPL_HAS_ASSIMP3 AND ASSIMP_LIBRARIES AND ASSIMP_INCLUDE_DIRS)
    set(OMPL_HAS_ASSIMP3)
endif()

find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARIES ASSIMP_INCLUDE_DIRS)
