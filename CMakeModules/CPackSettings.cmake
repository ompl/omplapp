set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL) & front-end (OMPL-APP)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPLAPP_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPLAPP_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPLAPP_PATCH_VERSION}")

# component list
set(CPACK_COMPONENTS_ALL ompl python morse omplapp)
# display names for components
set(CPACK_COMPONENT_OMPL_DISPLAY_NAME "OMPL library, headers, and demos")
set(CPACK_COMPONENT_PYTHON_DISPLAY_NAME "Python bindings")
set(CPACK_COMPONENT_MORSE_DISPLAY_NAME "Blender/MORSE plugin")
set(CPACK_COMPONENT_OMPLAPP_DISPLAY_NAME "OMPL.app library and GUI")
# descriptions of components
set(CPACK_COMPONENT_MORSE_DESCRIPTION "The Blender/MORSE plugin allows one to plan paths using the MORSE robot simulator. MORSE is built on top of Blender and uses its built-in physics engine to compute physically realistic motions.")
set(CPACK_COMPONENT_OMPLAPP_DESCRIPTION "The OMPL.app library makes it easy to read meshes (using the Assimp library) and perform collision checking (using the FCL library). The GUI is built on top of this and makes it easy to perform geometric and control-based planning for rigid bodies.")
# intercomponent dependencies
set(CPACK_COMPONENT_PYTHON_DEPENDS ompl)
set(CPACK_COMPONENT_MORSE_DEPENDS python)
set(CPACK_COMPONENT_OMPLAPP_DEPENDS python)
# core library is required
set(CPACK_COMPONENT_OMPL_REQUIRED TRUE)

set(CPACK_SOURCE_IGNORE_FILES
    "/.hg"
    "/.vscode"
    "/build/"
    ".pyc$"
    ".pyo$"
    "__pycache__"
    ".so$"
    ".dylib$"
    ".orig$"
    ".log$"
    ".DS_Store"
    "/html/"
    "/bindings/"
    "TODO"
    "releaseChecklist.txt"
    "ompl.pc$"
    "create_symlinks.sh$"
    "uninstall_symlinks.sh$"
    "config.h$"
    ".registered$"
    "download.md$"
    "mainpage.md$"
    "binding_generator.py$"
    "install-ompl-ubuntu.sh$")
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
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "python${PYTHON_VERSION}, libboost-serialization-dev, libboost-filesystem-dev, libboost-system-dev, libboost-program-options-dev, libboost-test-dev, python-opengl, python-pyqt5.qtopengl, freeglut3-dev, libode-dev, libassimp-dev, libtriangle-dev, libccd-dev")
endif()

if(APPLE)
    set(CPACK_GENERATOR "PackageMaker;${CPACK_GENERATOR}")
endif()

if(WIN32)
    set(CPACK_GENERATOR "ZIP;${CPACK_GENERATOR}")
endif()

include(CPack)
