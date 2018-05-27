# **Extended Kalman Filter Project**

## Goals / Steps

In this project a extended kalman filter will be utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric.


## [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

## Other Important Dependencies

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

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Equations

Equations of EKF are illustrated in this [document](sensor-fusion-ekf-reference.pdf). The equations are written in c++ by using Eigen library for matrix manipulation.

## Running Results in Simulator

Finally the Extended Kalman Filter should be run against Dataset 1 in the simulator which is the same as [data/obj_pose-laser-radar-synthetic-input.txt](data/obj_pose-laser-radar-synthetic-input.txt). The positions output by Kalman Filter are compared to ground truth data. The px, py, vx, and vy RMSE is [0.0974, 0.0855, 0.4517, 0.4404].
