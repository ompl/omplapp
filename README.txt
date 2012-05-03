Visit http://ompl.kavrakilab.org/installation.html for
detailed installation instructions.

OMPL.app has the following required dependencies:
 * Boost (version 1.44 or higher)
 * CMake (version 2.8.2 or higher)
 * Python including libraries and header files)
 * PyQt4
 * PyOpenGL
 * assimp
 * PQP
 * Py++ (version from repository, needed to generate Python bindings)

The following dependencies are optional:
 * ODE (needed to compile support for planning using the Open Dynamics Engine)
 * Doxygen (needed to create a local copy of the documentation at
   http://ompl.kavrakilab.org)

Once dependencies are installed, you can build OMPL.app on Linux and OS X.
Go to the top-level directory of OMPL.app and type the following commands:
   mkdir -p build/Release
   cd build/Release
   cmake -DCMAKE_BUILD_TYPE=Release ../..
   make -j 4 # replace "4" with the number of cores on your machine

