set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL) & front-end (OMPL-APP)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPLAPP_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPLAPP_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPLAPP_PATCH_VERSION}")

set(CPACK_SOURCE_IGNORE_FILES
    "/.hg"
    "/build/"
    ".pyc$"
    ".pyo$"
    "__pycache__"
    ".so$"
    ".dylib$"
    ".orig$"
    ".DS_Store"
    ".tmproj$"
    ".tm_properties"
    "mkwebdocs.sh"
    "/html/"
    "/bindings/"
    "TODO"
    "/external/assimp"
    "/pqp-1.3"
    "releaseChecklist.txt"
    "exposed_decl.pypp.txt"
    "ompl.pc$"
    "installPyPlusPlus.bat$"
    "installPyPlusPlus.sh$"
    "config.h$"
    ".tgz$")
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_GENERATOR "TGZ")

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(CPACK_GENERATOR "DEB;${CPACK_GENERATOR}")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
      set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
  endif()
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
      set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
  endif()
  execute_process(COMMAND "/usr/bin/lsb_release" "-rs"
      OUTPUT_VARIABLE UBUNTU_RELEASE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CPACK_PACKAGE_FILE_NAME "omplapp_${OMPL_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}-Ubuntu${UBUNTU_RELEASE}")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "python${PYTHON_VERSION}, libboost-all-dev, python-qt4-dev, python-qt4-gl, freeglut3-dev, libode-dev, libassimp-dev, libflann-dev")
endif()

if(WIN32)
  set(CPACK_GENERATOR "ZIP;${CPACK_GENERATOR}")
endif()

include(CPack)

