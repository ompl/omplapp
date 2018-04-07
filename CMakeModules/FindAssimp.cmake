include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(ASSIMP assimp)
    if(ASSIMP_LIBRARIES AND NOT ASSIMP_INCLUDE_DIRS)
        set(ASSIMP_INCLUDE_DIRS "/usr/include")
    endif()
else()
    # vcpkg installs cmake config files for assimp
    find_package(ASSIMP CONFIG)
endif()

find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARIES ASSIMP_INCLUDE_DIRS)
