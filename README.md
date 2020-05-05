The Open Motion Planning Library (OMPL)
=======================================

Linux [![Build Status](https://travis-ci.org/ompl/omplapp.svg?branch=master)](https://travis-ci.org/ompl/omplapp)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/dyu0y627hkp8tp6h/branch/master?svg=true)](https://ci.appveyor.com/project/mamoll/omplapp/branch/master)

This is OMPL.app, an extended version of OMPL that adds support for mesh
loading and collision checking as well as a simple GUI.

Visit the [OMPL.app installation page](http://ompl.kavrakilab.org/installation.html) for
detailed installation instructions.

OMPL.app has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.58 or higher)
* [CMake](https://www.cmake.org) (version 3.5 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)
* [Assimp](http://assimp.org) (version 3.0.1270 or higher)
* [FCL](https://github.com/flexible-collision-library/fcl) (version 0.3.1 or higher)

The following dependencies are optional:

* [PyQt](https://www.riverbankcomputing.co.uk/software/pyqt/download5) (for GUI)
* [PyOpenGL](https://pyopengl.sourceforge.net/) (for GUI)
* [Py++](https://bitbucket.org/ompl/ompl/src/tip/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org)

Once dependencies are installed, you can build OMPL.app on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL.app and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want to use the GUI or Python bindings
    make -j 4 # replace "4" with the number of cores on your machine
