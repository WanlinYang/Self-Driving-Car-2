# **Unscented Kalman Filter Project**

## Goals

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points

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
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Models

The motion model is Constant Turn Rate and Velocity Magnitude (CTRV) Model. The first prediction step is made by applying sigma points over the nonlinear model. In the second update step, lidar data is updated by Kalman Filter (linear), and radar data by Unscented Kalman Filter (nonlinear).

## Running Results in Simulator

Finally the Unscented Kalman Filter should be run against Dataset 1 in the simulator which is the same as [data/obj_pose-laser-radar-synthetic-input.txt](data/obj_pose-laser-radar-synthetic-input.txt). The positions output by Kalman Filter are compared to ground truth data. The px, py, vx, and vy RMSE is [0.0746, 0.0827, 0.3475, 0.2478].
