The Open Motion Planning Library (OMPL)
=======================================

This is OMPL.app, an extended version of OMPL that adds support for mesh
loading and collision checking as well as a simple GUI.

Visit the [OMPL.app installation page](http://ompl.kavrakilab.org/installation.html) for
detailed installation instructions.

OMPL.app has the following required dependencies:

* [Boost](http://www.boost.org) (version 1.44 or higher)
* [CMake](http://www.cmake.org) (version 2.8.2 or higher)
* [Assimp](http://assimp.sourceforge.net)
* [PQP](http://gamma.cs.unc.edu/SSV/)
* [FCL](https://kforge.ros.org/projects/fcl)

OMPL.app's build system will attempt to automatically download and build
Assimp, PQP, and FCL if not already installed.

The following dependencies are optional:

* [PyQt4](http://www.riverbankcomputing.co.uk/software/pyqt/download) (for GUI)
* [PyOpenGL](http://pyopengl.sourceforge.net/) (for GUI)
* [Py++](https://sourceforge.net/projects/pygccxml) (version from repository, needed to generate Python bindings)
* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  http://ompl.kavrakilab.org)

Once dependencies are installed, you can build OMPL.app on Linux, OS X,
and MS Windows. Go to the top-level directory of OMPL.app and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next two steps are optional
    make installpyplusplus && cmake . # download & install Py++
    make update_bindings # if you want to use the GUI or Python bindings
    make -j 4 # replace "4" with the number of cores on your machine

