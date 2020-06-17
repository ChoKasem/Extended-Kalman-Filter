**# Extended Kalman Filter**

**Goal** 

The goals / steps of this project are the following:

* Wrote and compile the code for of car simulation that contain laser and radar sensor data
* Perform Sensor Fusion to enable the use of both laser and radar sensor data for Kalman Filter
* Implement Extended Kalman Filter from both sensors to predict the state of the car [px, py, vx, vy]
* Determine the Root Mean Square Error (RMSE) of the predition and the ground truth state of the car
* Analyze the accuracy of the Kalman Filter
* Summarize the results with a written report

## 1. Setup

### 1. UWebSocketIO and Simulation
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. For Linux and Mac user, run `install-ubuntu.sh` or `install-mac.sh` script to download the uWebSocketIO. For Window user, download ubuntu bash from microsoft store and run `install-ubuntu.sh` script.

Download the simulation from this repository: https://github.com/udacity/self-driving-car-sim/releases/

### 2. Other Dependencies
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

### 3. Build and Compile

1. Clone this repository
2. From the root directory of the project, make a build directory `mkdir build && cd build` 
3. Compile the project `cmake .. && make`
4. From the build command, run the ExtendedKF file `./ExtendedKF`
5. Run the simulation for the EKF project, the command prompt should say `Connected!!!`

## 2. Result

