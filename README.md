# SeniorProject2018-2019

Dakota Adra and Eric Jones '19<br>
Bradley University - Dept. of Electrical and Computer Engineering<br>
1501 W Bradley Ave, Peoria, IL 61625 USA<br>

## At this point, ALL CODE in this repository is a work in progress and subject to change.<br>

bin
* noros
  * This folder contains some testing code in C++ that we used to test our system without ROS.
* test
  * This folder contains testing code in C that we used to test our PID tuning outside of ROS.

implement_test
* include
  * This folder contains a temporary file just in case we need to use an additional library outside of librobotcontrol.
* launch
  * This folder contains a temporary file just in case we want to use launch files later.
* src
  * This folder contains all of the source code that CMakeLists.txt compiles via CMake (i.e. the catkin build system).
    
eduMODClient - Copy.cpp<br>
* This code is the final version of the control code running on the robot. It contains a well-tuned PID controller and dead-reckoning algorithm to send its estimated position over ROS to a subscriber running on Matlab. See the code for additional documentation. 
