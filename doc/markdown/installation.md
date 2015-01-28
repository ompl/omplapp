# Installation

If you use Linux or OS X, then all dependencies can be installed either through a package manager or by OMPL's build system. In other words, you probably don't have to compile dependencies from source.

To compile OMPL and OMPL.app the following two packages are required:

- [Boost], version 1.48 or higher, and
- [CMake], version 2.8.7 or higher.

The build system includes a [number of options](buildOptions.html) that you can enable or disable. To be able to generate python bindings you need to install the [Python] library and header files.

Finally, to use the GUI the following dependencies are required (in addition to the ones above):

- [PyQt4] or [PySide][] (which, in turn, require [Qt4]) and
- [PyOpenGL].

Below are more detailed installation instructions for [Linux](#install_linux), [OS X](#install_osx), and [Windows](#install_windows).

\note If you are using [ROS], please see [MoveIt!][moveit].

<div class="btn-group">
  <a class="btn btn-default" href="#install_linux">Linux</a>
  <a class="btn btn-default" href="#install_osx">OS X</a>
  <a class="btn btn-default" href="#install_windows">Windows</a>
</div>


# Installation on Linux {#install_linux}

Below are installation instructions for Ubuntu Linux. Similar steps can be taken for other distributions, but package names may not be identical.


## Ubuntu Linux

- Install Boost, CMake, Python, PyQt4, PyOpenGL, and pkg-config.

      sudo apt-get install libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev

  Some versions of Linux offer multiple versions of Boost. It is strongly advised to install only one version.
- If the rendering in the OMPL.app GUI seems sluggish, you may want to install [PyOpenGL-accelerate](http://pypi.python.org/pypi/PyOpenGL-accelerate) to enable OpenGL hardware acceleration.
- To be able to generate documentation and build the OpenDE extension, the following packages are also needed:

      sudo apt-get install doxygen graphviz libode-dev

- Create a build directory and run cmake:

      cd omplapp
      mkdir -p build/Release
      cd build/Release
      cmake ../..

- If you want Python bindings or a GUI, type the following two commands:

      make installpyplusplus && cmake . # download & install Py++
      make update_bindings

- Compile OMPL.app by typing \c make.
- Optionally, run the test programs by typing <tt>make test</tt>.
- Optionally, generate documentation by typing <tt>make doc</tt>.
- If you need to install the library, you can type <tt>sudo make install</tt>. The install location is specified by <tt>CMAKE_INSTALL_PREFIX</tt>. If you install in a non-standard location, you have to set the environment variable PYTHONPATH to the directory where the OMPL python module is installed (e.g., $HOME/lib/python2.7/site-packages).

## Fedora Linux

The installation instructions for Fedora Linux are mostly the same as for Ubuntu Linux, although the packages have slightly different names. On Fedora, you can install the dependencies like so:

      sudo yum install boost-devel cmake python-devel PyQt4 PyOpenGL assimp-devel mesa-libGL-devel libccd-devel assimp-devel

The optional dependencies can be installed like so:

      sudo yum install doxygen graphviz ode-devel

The build steps are the same as for Ubuntu Linux:

      cd omplapp
      mkdir -p build/Release
      cd build/Release
      cmake ../..
      make installpyplusplus && cmake . # download & install Py++
      make -j 4 update_bindings
      make -j 4


# Installation on Mac OS X {#install_osx}

It is easiest to install the OMPL.app through [MacPorts], a package manager for OS X. However, if you feel adventurous, it is possible to install OMPL.app's dependencies with [HomeBrew](#install_homebrew) and compile OMPL.app yourself.


## MacPorts {#install_macports}

- Install [MacPorts].
- If you do not need to modify or see the source code of OMPL.app, then the easiest way to install OMPL.app is with the MacPorts \c port command:

      sudo port sync \; install ompl +app

  This is it. You are done. It will take quite a while, though, to download, build, and install OMPL.app and all its dependencies. It is recommended to let this running overnight. The GUI is invoked from the command line by \c /opt/local/bin/ompl_app. Demo programs and input files for the GUI can be found in \c /opt/local/share/ompl.

- If you downloaded the source distribution of OMPL.app, then you need to install the dependencies Boost, CMake, assimp, FCL, OpenDE, PyQt4, PyOpenGL, and Py++. If you have MacPorts installed, type the following:

      sudo port sync
      sudo port install boost cmake assimp fcl ode py27-pyqt4 py27-opengl py27-pyplusplus

  The commands above can take quite a while to complete. It is recommended to let this run overnight. If you do not care for the Python bindings or the GUI, you can reduce the total number of dependencies that need to be installed significantly by typing the following commands instead of the ones above:

      sudo port sync
      sudo port install boost cmake assimp fcl ode

- It is __very__ important that you use the same installed version of Python for all dependencies and OMPL.app. If you are using MacPorts, then you __must__ use the MacPorts version of python 2.7 (most likely installed in <tt>/opt/local/bin</tt>). To make this version the default python version, make sure <tt>/opt/local/bin</tt> appears before <tt>/usr/bin</tt> in your PATH. You can add a line like this to your <tt>${HOME}/.bash_profile</tt>:

      export PATH=/opt/local/bin:/opt/local/sbin:$PATH

  Next, execute the following command:

      sudo port select python python27

  You can check if the MacPorts-installed python 2.7 is the default one by typing the following command:

      ls -l `which python`

  This should output something like this:

      lrwxr-xr-x  1 root  admin  24 May 22 10:45 /opt/local/bin/python -> /opt/local/bin/python2.7

- To be able to generate documentation, Graphviz and Doxygen also needed:

      sudo port install graphviz doxygen

- Create a build directory and run cmake:

      cd omplapp
      mkdir -p build/Release
      cd build/Release
      cmake ../..

- If you want Python bindings, type the following command:

      make update_bindings

- Compile OMPL.app by typing \c make.
- Optionally, run the test programs by typing <tt>make test</tt>.
- Optionally, generate documentation by typing <tt>make doc</tt>.
- If you need to install the library, you can type <tt>sudo make install</tt>. The install location is specified by <tt>CMAKE_INSTALL_PREFIX</tt>. If you install in a non-standard location, you have to set the environment variable PYTHONPATH to the directory where the OMPL python module is installed (e.g., $HOME/lib/python2.7/site-packages).


## Homebrew {#install_homebrew}

_Thanks to [Andrew Dobson](https://plus.google.com/104214233559576935970/about) for these instructions!_ __These instructions are somewhat experimental, however, and we have not tested them ourselves.__ Email us if you have suggestions to improve these instructions.

- Install [Homebrew].
- Run <tt>brew doctor</tt> to make sure that everything is ready to go.  If not, follow its instructions until it is ready.
- Type the following commands:

      easy_install pip
      brew install boost cmake assimp pyqt ode
      pip install PyOpenGL PyOpenGL-accelerate

- To be able to generate documentation, Doxygen and Graphviz are also needed:

      brew install doxygen graphviz

- Create a build directory and run cmake:

      cd omplapp
      mkdir -p build/Release
      cd build/Release
      cmake ../..

- If you want Python bindings, type the following commands:

      make installpyplusplus
      cmake .
      make update_bindings

- Compile OMPL.app by typing \c make. If you do not already have the FCL package installed, then the OMPL.app build system will download and compile it for you.
- Optionally, run the test programs by typing <tt>make test</tt>.
- Optionally, generate documentation by typing <tt>make doc</tt>.
- If you need to install the library, you can type <tt>sudo make install</tt>. The install location is specified by <tt>CMAKE_INSTALL_PREFIX</tt>. If you install in a non-standard location, you have to set the environment variable PYTHONPATH to the directory where the OMPL python module is installed (e.g., $HOME/lib/python2.7/site-packages).


# Installation on Windows {#install_windows}

\note It is possible to run OMPL and OMPL.app natively on Windows, although it must be stressed that __extensive testing on Windows is not performed__ at this time, and running OMPL.app on Windows is considered <i>highly</i> experimental. It is <i>much</i> easier to install [VirtualBox], create an Ubuntu virtual machine, and follow the Ubuntu installation directions above.

For best performance, the [MinGW] compiler is recommended. Visual Studio can also be used to build the core OMPL and OMPL.app libraries, but currently it is not possible to generate the python bindings for OMPL with this compiler. However, if the bindings are generated with MinGW, the bindings can be compiled by Visual Studio with some minor tweaks to the code (not recommended, unless you are an experienced Windows developer).


## Required Dependencies

- [CMake], version 2.8.2 or higher,
- [MinGW][] (recommended) or the Visual Studio compiler, and
- [Boost], version 1.44 or higher.

  It is recommended to make a complete Boost compilation from source. If using Visual Studio, this process can be automated using the [BoostPro](http://www.boostpro.com/download) installer. Once complete, set the environment variables <tt>BOOST_ROOT</tt> and <tt>BOOST_LIBRARYDIR</tt> to the locations where Boost and its libraries are installed. The default locations are <tt>C:\\Boost</tt> and <tt>C:\\Boost\\lib</tt>. Ensure that <tt>BOOST_LIBRARYDIR</tt> is also in the system PATH so that any necessary Boost dlls are loaded properly at runtime.

- [Assimp](http://assimp.sourceforge.net) If not already installed, the last release (version 2.0) is downloaded and compiled automatically via CMake. If using Visual Studio 2010, this version will __NOT__ compile. It is recommended to checkout Assimp from their svn repository and install assimp.lib/dll manually if using VS2010. Ensure that Assimp is installed and that the include and lib directories are in the system PATH.


## Optional Dependencies (for Python bindings and OMPL.app GUI)

- A __32-bit__ version of [Python] 2.7.  Ensure that this is installed __before building Boost__ so that Boost.Python is properly compiled.
- Ensure that Python is added to the system <tt>PATH</tt>.
- Py++: To generate the Python bindings, Py++ and its dependencies must be installed. A batch file has been included to automate this process (analogous to the Linux/Mac installation) that can be executed via cmake. Instructions can be found [here](installPyPlusPlus).  Note that this process assumes the MinGW compiler, and installs gccxml to <tt>C:\\gccxml</tt>.  You will need to be in a shell with administrator privileges to execute this batch file.  Once installed, it is recommended that you open a new shell to realize the new environment settings.
- [PyQt4] and [PyOpenGL] must be installed to run the OMPL.app gui.
- [pkg-config](http://ftp.gnome.org/pub/gnome/binaries/win32/dependencies/) must be installed for the collision checking library (FCL).


## Build
- Once the dependencies are installed, CMake can be used to generate MinGW makefiles or a Visual Studio solution by specifying a specific GENERATOR:

      cd omplapp
      mkdir build
      cd build
      mkdir Release
      cd Release
      cmake -G "GENERATOR" ../.. [-DCMAKE_INSTALL_PREFIX=/path/to/install]

The CMAKE_INSTALL_PREFIX variable is set to <tt>C:\\Program Files (x86)\\omplapp</tt> by default.


### MinGW

- The CMake generator for MinGW is <tt>"MinGW Makefiles"</tt>
- To generate the python bindings (optional), execute the update_bindings make command before compiling:

      mingw32-make update_bindings

  __Note:__ <tt>update_bindings</tt> is _never_ run automatically. If you change any of the OMPL header files, you need to regenerate the bindings for the changes to be reflected in the Python modules. See also the [more detailed documentation on generating python bindings](\ref updating_python_bindings).
- Use <tt>mingw32-make</tt> to build OMPL and OMPL.app.
- If you wish to install OMPL and OMPL.app, use the install command to copy the binaries, demo code, and other resources to the cmake install prefix.  Note that you will need to be in a shell with administrator privileges to install to the default directory.

      mingw32-make install

- Make sure to add the install path's \\lib subdirectory to the PATH so that the DLLs are found when code is loaded.


### Visual Studio

- The CMake generator for Visual Studio depends on the version of Visual Studio to generate a solution for. The generator for VS 2010 is <tt>"Visual Studio 10"</tt>, and the generator for VS 2008 is <tt>"Visual Studio 9 2008"</tt>. Consult the CMake documentation for other generators.
- Open omplapp.sln and build the solution.  Two static libraries will be created (ompl.lib and ompl_app.lib) to link your code against, as well as several demo programs.
- You can install OMPL and OMPL.app by building the "INSTALL" project inside of the solution.  Note that this will attempt to copy files to <tt>C:\\Program Files (x86)\\omplapp</tt> (the default).  The installation will fail unless Visual Studio is opened with administrator privileges, or a non-system install prefix is specified when cmake is run.

[boost]: http://www.boost.org
[cmake]: http://www.cmake.org
[python]: http://www.python.org
[qt4]: http://qt-project.org
[pyqt4]: http://www.riverbankcomputing.co.uk/software/pyqt/download
[pyside]: http://www.pyside.org
[pyopengl]: http://pyopengl.sourceforge.net
[ros]: http://www.ros.org
[moveit]: http://moveit.ros.org
[macports]: http://www.macports.org
[homebrew]: http://mxcl.github.com/homebrew
[mingw]: http://www.mingw.org
[virtualbox]: http://www.virtualbox.org
