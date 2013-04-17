include(FindPackageHandleStandardArgs)

find_library(PQP_LIBRARY PQP DOC "Location of PQP proximity query library")
find_path(PQP_INCLUDE_DIR PQP.h PATH_SUFFIXES "PQP"
    DOC "Location of PQP proximity query header files")

if(PQP_LIBRARY AND PQP_INCLUDE_DIR)
    set(OMPL_HAS_PQP 1)
endif()

find_package_handle_standard_args(PQP DEFAULT_MSG PQP_LIBRARY PQP_INCLUDE_DIR)
mark_as_advanced(PQP_LIBRARY PQP_INCLUDE_DIR)
