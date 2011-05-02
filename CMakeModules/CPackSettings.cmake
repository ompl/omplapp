set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL) & front-end (OMPL-APP)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPLAPP_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPLAPP_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPLAPP_PATCH_VERSION}")

set(CPACK_SOURCE_IGNORE_FILES
    "/.hg"
    "/.svn/"
    "/build/"
    ".pyc$"
    ".pyo$"
    ".so$"
    ".dylib$"
    ".md5$"
    "/blueprint/"
    ".DS_Store"
    ".tmproj$"
    "mkwebdocs.sh"
    "/html/"
    "TODO"
    "/external/assimp"
    "/pqp-1.3"
    "releaseChecklist.txt")
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_GENERATOR "TGZ;ZIP")

include(CPack)

