# - Find Drawstuff (ODE Utility Lib)

# This module finds if the drawstuff library, part of Open Dynamics Engine (ODE).
#
# The following variables are set:
#  DRAWSTUFF_LIBRARY     = location of drawstuff library
#  DRAWSTUFF_INCLUDE     = include path for drawstuff

include(FindPackageHandleStandardArgs)

find_package(ODE QUIET)

if (ODE_FOUND)

  # find the header file
  find_path(DRAWSTUFF_INCLUDE drawstuff.h PATHS ${_ODE_INCLUDE_HINTS} ${ODE_INCLUDE} PATH_SUFFIXES drawstuff include/drawstuff)
  if (DRAWSTUFF_INCLUDE)
    string(REGEX REPLACE "/drawstuff$" "" DRAWSTUFF_INCLUDE ${DRAWSTUFF_INCLUDE})
  endif()

  # find the lib
  find_library(DRAWSTUFF_LIBRARY drawstuff PATHS ${_ODE_LIB_HINTS} PATH_SUFFIXES lib drawstuff/src/.libs src/.libs)
  
endif()

find_package_handle_standard_args(Drawstuff DEFAULT_MSG DRAWSTUFF_INCLUDE DRAWSTUFF_LIBRARY)
mark_as_advanced(DRAWSTUFF_LIBRARY DRAWSTUFF_INCLUDE)
