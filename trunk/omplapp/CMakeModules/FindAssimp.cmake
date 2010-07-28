# - Try to find Assimp
# Once done this will define
#
#  ASSIMP_FOUND - system has Assimp
#  ASSIMP_INCLUDE_DIR - the Assimp include directory
#  ASSIMP_LIBRARIES - Link these to use Assimp
#


FIND_PATH(ASSIMP_INCLUDE_DIR NAMES assimp.h
  PATHS
  ${PROJECT_SOURCE_DIR}/external/assimp
  $ENV{ASSIMP_PATH}
  PATH_SUFFIXES include/
)
 
IF(ASSIMP_INCLUDE_DIR)
  message(STATUS "Found assimp header files")
ENDIF(ASSIMP_INCLUDE_DIR)

FIND_LIBRARY(ASSIMP_LIBRARIES
  NAMES
  assimp
  PATHS
  ${PROJECT_SOURCE_DIR}/external/assimp/lib
  $ENV{ASSIMP_PATH}
)

IF(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)
   SET(ASSIMP_FOUND TRUE)
   message(STATUS "Found assimp lib")
ELSE(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)
  message(STATUS "assimp not found")
ENDIF(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARIES)


