# f-control

Control engeneering c++ library.

# Dependencies

Build system is configured to search for the following dependencies as required (will throw error if not installed).

* fftw
* eigen

You can install under a debian based linux with:
```
sudo apt install fftw-dev libeigen3-dev
```
For ubuntu 18.04 LTS you need FFTW3 library instead, therefore:
```
sudo  apt-get install libfftw3-dev libeigen3-dev
```

# How to use

This library is intended for the use with CMake build system.

## In project
For in project CMake build, find all the include and link directories in variables:

```cmake
SUBDIR_INCLUDE_DIRECTORIES
SUBDIR_LINK_NAMES
```

Then, assuming the library is placed at "${PROJECT_SOURCE_DIR}/lib/fcontrol/" (for example after clone with ``git clone https://gitlab.com/munoz.yanez/fcontrol.git ``, it is enough to add the following lines to CMakeLists.txt to add includes:

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fcontrol/)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})
```

Also after "add_executable( ${name} ${sourcefile} )" line, add the following to link the library:

```cmake
target_link_libraries( ${PROJECT_NAME} ${SUBDIR_LINK_NAMES} )
```
## As a system library
Use the usual CMake and make install commands in order to compile and install library on the system

```
git clone https://gitlab.com/munoz.yanez/fcontrol.git
mkdir .fcontrol-build
cd .fcontrol-build
cmake ../fcontrol
make
make install
```

Then use Findfcontrol.cmake file from ( https://gitlab.com/munoz.yanez/fcontrol-example/blob/master/cmake/Findfcontrol.cmake ) in order to use CMake find_package(fcontrol) directive.

You can also uninstall the library with:

```
make uninstall
```
inside .fcontrol-build directory.

See https://gitlab.com/munoz.yanez/fcontrol-example/blob/master/lib/CMakeLists.txt for an example of both cases.
