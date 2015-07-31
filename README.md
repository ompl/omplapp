The Open Motion Planning Library (OMPL)
=======================================

Linux [![Build Status](https://travis-ci.org/ompl/omplapp.svg?branch=master)](https://travis-ci.org/ompl/omplapp)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/dyu0y627hkp8tp6h/branch/master?svg=true)](https://ci.appveyor.com/project/mamoll/omplapp/branch/master)

This is OMPL.app, an extended version of OMPL that adds support for mesh
loading and collision checking as well as a simple GUI.

Visit the [OMPL.app installation page](http://ompl.kavrakilab.org/installation.html) for
detailed installation instructions.

OMPL.app has the following required dependencies:

* [Boost](http://www.boost.org) (version 1.48 or higher)
* [CMake](http://www.cmake.org) (version 2.8.7 or higher)
* [Assimp](http://assimp.sourceforge.net)
* [FCL](http://gamma.cs.unc.edu/FCL)

OMPL.app's build system will attempt to automatically download and build
Assimp and FCL if not already installed.

The following dependencies are optional:

* [PyQt4](http://www.riverbankcomputing.co.uk/software/pyqt/download) (for GUI)
* [PyOpenGL](http://pyopengl.sourceforge.net/) (for GUI)
* [Py++](https://bitbucket.org/ompl/pyplusplus) (needed to generate Python bindings)
* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  http://ompl.kavrakilab.org)
* [Eigen](http://eigen.tuxfamily.org) (needed for an informed sampling technique to improve the optimization of path length)

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

