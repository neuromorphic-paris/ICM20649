# ICM20649

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg?style=flat-square)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

Simple library to interface with ICM20649 IMU using I2C communication protocol.

# Requirements
This code was tested on Ubuntu 16.04, 18.04 and 20.04 distros.

## Dependencies
CMake: <https://cmake.org/>
  ```
  sudo apt install cmake
  ```

Clang-format:
  ```
  sudo apt install clang-format
  ```

I2C tools:
  ```
  sudo apt install libi2c-dev i2c-tools
  ```

## Install
Clone the repository
  ```
  git clone git@github.com:neuromorphic-paris/ICM20649.git
  ```

Build and compile the executables:
  ```
  cd ICM20649
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make
  ```

### Tests
To run the tests, you can turn `ON` the `ICM20649_BUILD_TEST` flag and compile everything again.
Then, just run:
  ```
  make test
  ```

### Documentation
To build the documentation you need Doxygen installed and turn `ON` the `ICM20649_BUILD_DOC` flag.
Then, just run:
  ```
  make doc
  ```

## Example
An [example](src/display_measurements.cpp) executable is provided that displays the measurements from the IMU accelerometer, gyroscope and temperature sensors.
To run it, from the repository root directory type:
  ```
  cd build/src
  ./display_measurements
  ```

# License
The ICM20649 code is licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
Commercial usage is not permitted.
