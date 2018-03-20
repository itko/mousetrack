# Mousetrack

[![Build Status](https://travis-ci.org/Fluci/mousetrack.svg?branch=master)](https://travis-ci.org/Fluci/mousetrack)

This project will be awesome!

## Dependencies

TL;DR: Convenient one-liner for Ubuntu: `$ sudo apt install -y cmake libboost-all-dev libopencv-dev libeigen3-dev`

- CMake: `$ sudo apt install cmake`
- Boost.Test: `$ sudo apt install libboost-all-dev`
- [OpenCV](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html): `$ sudo apt install libopencv-dev`
- Eigen: `$ sudo apt install libeigen3-dev`


If operations run slow, you can try reinstalling OpenCV with these additional dependencies:
- Atlas: `$ sudo apt-get install libatlas-base-dev gfortran`

## Build project

TL;DR: `$ mkdir build && cd build && cmake .. & make -j7`

We use CMake as main building tool. CMake uses one or more input files (called `CMakeLists.txt`), to understand how to compile the project.
You can find those files in the root of the project and in the `lib/` and `app/` folders.

With CMake, you usually create an empty build folder. From that folder you can run `$ cmake ..` to run the configuration which creates, for example, Makefiles.
Those Makefiles can be executed with `$ make` which compiles the code, and creates executables/libraries/etc.

CMake supports to build release and debug versions. 

This project uses `build/` as a release folder and `build-debug` for a debuggable build. 

Run `$ ./create_cmake.sh` to set up both folders automatically. Note: you still need to run `make` in each folder separately to generate the binaries.


## Documentation

You can generate HTML/LaTex/... documentation automatically from source code.
For this you need to install [Doxygen](http://www.stack.nl/~dimitri/doxygen/). 
Doxygen can also create nice class hierarchy diagrams and similar things, for this [Dot](http://www.graphviz.org/download/) from graphviz is needed.

If you've installed Doxygen successfully, you can run `$ doxygen` in the root of the project.

All generated documentation files will be placed in the folder `docs/`.

