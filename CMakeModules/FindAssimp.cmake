include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(ASSIMP assimp)
else()
    # vcpkg installs cmake config files for assimp
    find_package(ASSIMP CONFIG)
endif()

find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARIES ASSIMP_INCLUDE_DIRS)
