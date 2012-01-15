# - Find Drawstuff (OPENDE Utility Lib)

# This module finds if the drawstuff library, part of Open Dynamics Engine (OPENDE).
#
# The following variables are set:
#  DRAWSTUFF_LIBRARY     = location of drawstuff library
#  DRAWSTUFF_INCLUDE_DIR = include path for drawstuff

include(FindPackageHandleStandardArgs)

find_package(OpenDE QUIET)

if (OPENDE_FOUND)
  # find the header file
  find_path(DRAWSTUFF_INCLUDE_DIR drawstuff.h PATHS ${_OPENDE_INCLUDE_HINTS} ${OPENDE_INCLUDE_DIR}
      PATH_SUFFIXES drawstuff include/drawstuff
       DOC "Location of OpenDE's drawstuff header files")
  if (DRAWSTUFF_INCLUDE_DIR)
    string(REGEX REPLACE "/drawstuff$" "" DRAWSTUFF_INCLUDE_DIR "${DRAWSTUFF_INCLUDE_DIR}")
  endif()

  # find the library
  find_library(DRAWSTUFF_LIBRARY drawstuff PATHS ${_OPENDE_LIB_HINTS}
      PATH_SUFFIXES lib drawstuff/src/.libs src/.libs
      DOC "Location of OpenDE's drawstuff library")
endif()

find_package_handle_standard_args(Drawstuff DEFAULT_MSG DRAWSTUFF_INCLUDE_DIR DRAWSTUFF_LIBRARY)
mark_as_advanced(DRAWSTUFF_LIBRARY DRAWSTUFF_INCLUDE_DIR)
