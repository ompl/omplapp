# - Find Drawstuff (ODE Utility Lib)

# This module finds if the drawstuff library, part of Open Dynamics Engine (ODE).
#
# The following variables are set:
#  DRAWSTUFF_LIBRARY     = location of drawstuff library
#  DRAWSTUFF_INCLUDE_DIR = include path for drawstuff

include(FindPackageHandleStandardArgs)

find_package(ODE QUIET)

if (ODE_FOUND)
  # find the header file
  find_path(DRAWSTUFF_INCLUDE_DIR drawstuff.h PATHS ${ODE_INCLUDE_DIRS}
      PATH_SUFFIXES drawstuff
       DOC "Location of ODE's drawstuff header files")
  if (DRAWSTUFF_INCLUDE_DIR)
    string(REGEX REPLACE "/drawstuff$" "" DRAWSTUFF_INCLUDE_DIR "${DRAWSTUFF_INCLUDE_DIR}")
  endif()

  # find the library
  find_library(DRAWSTUFF_LIBRARY drawstuff PATHS ${ODE_LIBRARY_DIRS}
      DOC "Location of ODE's drawstuff library")
endif()

find_package_handle_standard_args(Drawstuff DEFAULT_MSG DRAWSTUFF_INCLUDE_DIR DRAWSTUFF_LIBRARY)
mark_as_advanced(DRAWSTUFF_LIBRARY DRAWSTUFF_INCLUDE_DIR)
