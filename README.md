# Extended Kalman Filter Project

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements and obtaining RMSE values that are lower that the tolerance outlined in the project [rubric](https://review.udacity.com/#!/rubrics/748/view).


[//]: # (Image References)

[image1]: ./images/radar_and_lidar.png "Both Radar and Lidar sensors"
[image2]: ./images/lidar_only.png "Lidar sensor only"
[image3]: ./images/radar_only.png "Radar sensor only"

## Results

In order to validate the Extended Kalman Filter performance, the filter was applied in three scenarios. While using: both the radar and lidar sensors, only the lidar sensor and only the radar sensor. After comparing the three respective oucomes we validate that the fusion of both the radar and lidar sensors provided the best result.

![alt text][image1]

Simulation with both Radar and Lidar - On: [youtube video](https://youtu.be/UkB2onGEMqI)

![alt text][image2]

Simulation with only the Lidar - On: [youtube video](https://youtu.be/HWgJ2dBUb_o)

![alt text][image3]

Simulation with only the Radar - On: [youtube video](https://youtu.be/wNag24x1FrM)

The table below summarizes the results for the three runs. It is clear that sensor fusion provided the best results for all the RMSE values (X, Y, VX, VY):

| Sensor  | X         | Y           | VX        | VY         |
|----------|---------|----------|----------|----------|
| Both     | **0.0982**  | **0.0897**  | **0.3929** |  **0.4075** |
| Lidar     | 0.1526  |  0.1213 | 0.7062 |  0.6030 |
| Radar   | 0.2271  | 0.3466  | 0.6587 |  0.7631 |

## Installation

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

## Important Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
   * Linux: make is installed by default on most Linux distros
   * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
   * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF`

```
$ mkdir build && cd build
$ cmake .. && make
$ ./ExtendedKF
```

Note: If you want to run the program with different sensor configuration see the `./src/constants.h` file.

## Data flow between the program and the Simulator

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator:

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

