# libcreate fork #
This is a modified version of libcreate usuable with the ROS 2 package create_robot_lyrical. The author is in no way associated with the original creator of this repo.


C++ library for interfacing with iRobot's Create 1 and 2 as well as most models of Roomba. [create_robot](http://wiki.ros.org/create_robot) is a [ROS](http://www.ros.org/) wrapper for this library.

* [Code API](http://docs.ros.org/noetic/api/libcreate/html/index.html)
* Protocol documentation:
  - [`V_1`](https://drive.google.com/file/d/0B9O4b91VYXMdUHlqNklDU09NU0k/view?usp=sharing&resourcekey=0-KxMpRPBMsGAj7eSYC_9ewA) (Roomba 400 series )
  - [`V_2`](https://drive.google.com/file/d/0B9O4b91VYXMdMmFPMVNDUEZ6d0U/view?usp=sharing&resourcekey=0-bqKH8xhtWdYtTik_LLWo9Q) (Create 1, Roomba 500 series)
  - [`V_3`](https://drive.google.com/file/d/0B9O4b91VYXMdSVk4amw1N09mQ3c/view?usp=sharing&resourcekey=0-rKvug2IzC7nj4zV31EJtww) (Create 2, Roomba 600-800 series)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](https://autonomy.cs.sfu.ca), [Simon Fraser University](http://www.sfu.ca))
* Contributors: [Mani Monajjemi](http:mani.im), [Ben Wolsieffer](https://github.com/lopsided98), [Josh Gadeken](https://github.com/process1183)

## Build Status ##

![Build Status](https://github.com/AutonomyLab/libcreate/workflows/Build%20and%20test/badge.svg)

## Dependencies ##

* CMake 3.20 or newer and a C++17 compiler
* Boost 1.69 or newer headers (Asio, System, and uBLAS)
* Platform threads, linked through CMake's `Threads::Threads`
* [Optional] [googletest](https://github.com/google/googletest)

### Install ###

        sudo apt-get install build-essential cmake libboost-dev

        # Optionally, install gtest for building unit tests
        sudo apt-get install libgtest-dev

This library is bundled in the `create_robot_lyrical` repository and includes
the CMake and Boost.Asio updates for Ubuntu 26.04 / ROS 2 Lyrical. Clone only
the parent repository; colcon discovers this package automatically. See
[the parent build instructions](../LYRICAL.md) and [upstream provenance](UPSTREAM.md).
Boost.System is header-only and Boost.Thread is unnecessary because the library
uses `std::thread`.

#### Serial Permissions ####

User permission is requried to connect to Create over serial. You can add your user to the dialout group to get permission:

        sudo usermod -a -G dialout $USER

Logout and login again for this to take effect.

## Build ##

Note, the examples found in the "examples" directory are built with the library.

#### cmake ####

        # From the create_robot_lyrical repository root:
        cd libcreate
        mkdir build && cd build
        cmake ..
        make -j

#### colcon ####

From the ROS 2 workspace root, build the bundled library and driver together:

        source /opt/ros/lyrical/setup.bash
        colcon build --symlink-install

## Running Tests ##

To run unit tests, execute the following in the build directory:

        make test

## Known Issues ##

* _Clock_ and _Schedule_ buttons are not functional. This is a known bug related to the firmware.
* Inaccurate odometry angle for Create 1 ([#22](https://github.com/AutonomyLab/libcreate/issues/22))
* Some 600 series models incorrectly report the OI Mode in their sensor stream ([create_robot #64](https://github.com/AutonomyLab/create_robot/issues/64))
  - To enable or disable the OI Mode reporting workaround, pass `true` or `false` to `setModeReportWorkaround()`
