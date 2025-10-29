# mixer_lib_ids

C++ library that allows you to communicate with the Mixer board.

## Getting the code

Clone the repository with "git clone https://dartleague.ddns.net:82/lbigazzi/comdemo_/-/tree/mixer_lib_ids"


## Build

In the root folder (The folder containing this file)

- mkdir build
- cd build
- cmake .. 
- make

To install, type the following command

- sudo make install 

# Run

At this point it is possible to run the application with the command.

- sudo ./ComDemo

To run correctely the software, you must be root users. Otherwise you get the following error message.

- kernel.sched_rt_runtime_us = -1
  pthread_create() failed (1 EPERM); are you sure you're root?
  You may also need to do:
  echo -1 > /proc/sys/kernel/sched_rt_runtime_us

In fact, the S.O cannot generate real time tasks without super user privileges.

# Test

The example code for using the library, once it is installed, is located inside the comDemo_testLib folder.

# ROS & ROS2 packages

To use the wrapper for ROS or ROS2, copy the files contained within the two folders into the respective ROS workspaces.

