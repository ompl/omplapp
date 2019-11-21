include(FindPackageHandleStandardArgs)

# vcpkg and MacPorts install cmake config files for fcl
find_package(fcl QUIET CONFIG)
if(TARGET fcl)
    get_target_property(FCL_INCLUDE_DIRS fcl INTERFACE_INCLUDE_DIRECTORIES)
    set(FCL_LIBRARIES "fcl")
else()
    find_package(PkgConfig)
    if(PKGCONFIG_FOUND)
        pkg_check_modules(FCL fcl>=0.3.1)
        if(FCL_LIBRARIES AND NOT FCL_INCLUDE_DIRS)
            set(FCL_INCLUDE_DIRS "/usr/include")
        endif()
    endif()
endif()

find_package_handle_standard_args(fcl DEFAULT_MSG FCL_LIBRARIES FCL_INCLUDE_DIRS)
