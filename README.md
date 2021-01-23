# CarND-Kidnapped-Vehicle-Project  
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview  
---
In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Description  
---
- This project can compile
- This particle filter has the desired accuracy
- Particles will be executed within the specified 100 seconds
- This code uses a particle filter to localize the robot

## Using Library
---
- [JSON for Modern C++](https://github.com/nlohmann/json)  
  Template header library for connecting in JSON format

- [uWebSockets](https://github.com/uNetworking/uWebSockets)  
  This is dynamic library for communicating with Term2.
  To communicate with Term2, you need to compile something with a specific commit ID, and you can build your environment with the following command:
```
./install-ubuntu.sh
```
## Basic Build And Run Instructions
---
Enter the command in the top directory of the project according to the following procedure.  

1. Compile the edited code
    1. mkdir build
    1. cd build
    1. cmake ..
    1. make
    1. ./particle_filter

    Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:
    1. ./clean.sh
    1. ./build.sh
    1. ./run.sh
1. Start Term2 Simulator

## Requirement
---
### Environment for starting an Extended Kalman Filter Project
- cmake >= 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)

### Simulator  
  This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Communication protocol with the simulator  
---
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.  

### INPUT: values provided by the simulator to the c++ program
// sense noisy position data from the simulator
["sense_x"]  
["sense_y"]  
["sense_theta"]  

// get the previous velocity and yaw rate to predict the particle's transitioned state  
["previous_velocity"]  
["previous_yawrate"]  

// receive noisy observation data from the simulator, in a respective list of x/y values
["sense_observations_x"]  
["sense_observations_y"]  

### OUTPUT: values provided by the c++ program to the simulator  
// best particle values used for calculating the error evaluation  
["best_particle_x"]  
["best_particle_y"]  
["best_particle_theta"]  

//Optional message data used for debugging particle's sensing and associations  
// for respective (x,y) sensed positions ID label  
["best_particle_associations"]

// for respective (x,y) sensed positions  
["best_particle_sense_x"] <= list of sensed x positions  
["best_particle_sense_y"] <= list of sensed y positions  

## Inputs to the Particle Filter
---
You can find the inputs to the particle filter in the `data` directory.

#### *The Map*  
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns  
1. x position  
2. y position  
3. landmark id  

## Submission  
---
- [writeup.md](./writeup.md)
- src/particle_filter.cpp
- src/particle_filter.h

## Code Style  
---
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Licence
---
[MIT](LICENSE)

## Author
---
[kumab2221](https://github.com/kumab2221)