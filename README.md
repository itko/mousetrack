# Mousetrack

[![Build Status](https://travis-ci.org/Fluci/mousetrack.svg?branch=master)](https://travis-ci.org/Fluci/mousetrack)

This project will be awesome!

## Dependencies

TL;DR: Convenient one-liner for Ubuntu:
``` bash
$ sudo apt install -y cmake libboost-all-dev libopencv-dev libeigen3-dev libpng16-dev
```

- CMake: `$ sudo apt install cmake`
- Boost.Test: `$ sudo apt install libboost-all-dev`
- [OpenCV](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html): `$ sudo apt install libopencv-dev`
- Eigen: `$ sudo apt install libeigen3-dev`
- We need to read PNG files: `$ sudo apt install libpng16-dev`


If operations run slow, you can try reinstalling OpenCV with these additional dependencies:
- Atlas: `$ sudo apt-get install libatlas-base-dev gfortran`

### Version requirements

This list is not complete, but might help you to finding possible problems when compiling for the first time.
Feel free to add or adjust entries on your own experience.

- g++ 4.9 or larger: we need C++11 with regex fully implemented, g++ 4.8 or bellow won't work
- gcc 5.4 is known to work
- clang 3.8 is known to work
- libpng 1.2 and 1.6 are known to work
- boost 1.56 and 1.58 are known to work
- OpenCV 3.3 is known to work
- Eigen 2.91 is known to work

## Build project

TL;DR:
``` bash
$ mkdir build && cd build && cmake .. & make -j7
```

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

## Logging

We use Boost.Test as logging framework.

For the moment, I recommend using `BOOST_LOG_TRIVIAL` ([official docs](http://www.boost.org/doc/libs/1_66_0/libs/log/doc/html/log/tutorial.html)). This should give us enough control while being easy to use.

To add logging to your source file, add this to your header:

``` C++
#include <boost/log/trivial.hpp>
```

Usage example directly taken from the docs:

``` C++
BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
```

`trace` is the severity level. There are six severity levels. While the "importance" increases going down the list, I'm not aware of exact definitions for the levels. I've hence added some rough thoughts as a starting point. In the end it probably won't matter to much which exact level is used.

- `trace`

    The finest level, it is allowed to print the input and output of each "step" of an algorithm.
    This means it is allowed to produce tons of output which could flood the logging system if turned on for a longer period of time.

    Example: We print the function state after each line of an algorithm.

- `debug`

    Information events needed to find bugs in a software.
    Think of it as checkpoints, one should be able to decide, if a certain part of the application state is still valid at this point. It should help to reduce the amount of code a bug could be hidden in.

    Example: Print the absolute paths of files to make sure they are correct.

- `info`

    The application behaves according to specifications but might surprise a user, since an edge case was encountered.

    Example: We scan a directory and the specification defines to ignore files we don't expect.
    A user might want to know if a file was ignored, and maybe why it was ignored.

- `warning`

    There might be a bug somewhere (for example a forgotten switch statement entry) or a base assumption was violated.

    Example: We performed a scan of a directory and stored some file paths in memory.
    When we come back at a later point, some files were removed by the user.

- `error`

    This obviously is a bug, but isn't urgent. For the moment we can recover from this event.

    Example: The contract of a function was violated.
    As a result, a transaction couldn't be performed.
    The violation of the contract clearly is a bug that needs to be fixed, but a transaction is allowed to fail, which gives the application the chance to recover.

- `fatal`

    Wake me (the developer) up in the middle of the night, this should never ever happen.
    This is an error with fatal consequences, it might cost a lot of money/lives/cute kittens if ignored.

    Example: A safety check in a financial application trading stocks failed, million of dollars could be lost.

