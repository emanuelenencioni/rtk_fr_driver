# ComDemo_

This project shows how the DART communication library works. In particular, the communication protocol is bidirectional: It is possible to send the command signals to the drone, vice versa the drone can send the telemetry to the Jetson module. 

# BUILDING

## Getting the code

Clone the repository with "git clone https://dartleague.ddns.net:82/lbigazzi/ComDemo_"


## Compilation

In the root folder (The folder containing this file)

- mkdir build
- cd build
- cmake .. 
- make -j4 

# Run

At this point it is possible to run the application with the command.

- sudo ./ComDemo

To run correctely the software, you must be root users. Otherwise you get the following error message.

- kernel.sched_rt_runtime_us = -1
  pthread_create() failed (1 EPERM); are you sure you're root?
  You may also need to do:
  echo -1 > /proc/sys/kernel/sched_rt_runtime_us

In fact, the S.O cannot generate real time tasks without super user privileges.
