# Chironix Practical Test

Pre-requisites to run packages within this repository:
* Ubuntu 18.04 and ROS Melodic
  * Please follow the [installation instructions here](http://wiki.ros.org/melodic/Installation/Ubuntu)
* Read the information pertaining to the description of each part of this practical test before cloning this repository

# Part 1: Simulation Robot Control

## Task Description
The Clearpath Husky has been controlled to perform two consecutive rotations. The first involves rotating the robot clockwise for 10 seconds, and the second rotating anti-clockwise for another 10 seconds before coming to a halt.

The node which performs the communication () of these commands to the robot has first been written in Python, then in C++. Both nodes perform identically.

A package named simulation_control has been included in this repository which contains all relevant files to perform this task.

Summary:
+ Spawn robot in simulated map
+ Move robot clockwise for 10 seconds
+ Move robot anti-clockwise for 10 seconds

Components:
+ 1x Python Node
+ 1x C++ Node
+ 1x Ubuntu System Service

# Part 2: Autonomous Navigation

## Task Description
The Clearpath Husky has been enabled to travel to specified goal coordinates within a simulated world using Gazebo. The Husky Velodyne VLP16 (LiDAR Sensor) has been enabled to develop a global and local costmap coinciding with the immediate known environment within the simulated world. Access to this map allows the robot to produce an estimated planned path using the move_base navigation stack develop for the Husky. The goal(s) correlate with the coordinates that are read from the .csv, with the path being planned to reach it with high accuracy. Topics exist within this navigation stack which develops a final status whether the goal can be reached or not.

The robot essentially navigates to each goal set in the .csv file (one goal per row), and will save the final location reached when finishing the set path and saving the final coordiantes reached and the status message of whether the goal was reached or not.

The node which performs the communication () of these commands to the robot has first been written in Python, then in C++. Both nodes perform identically.

A package named autonomous_nav has been included in this repository which contains all relevant files to perform this task.

Summary:
+ Spawn robot in simulated map
+ Create a costmap using lidar
+ Develop list of goals by reading .csv file
+ Use move_base for navigation and path planning
+ Save final destination when deliberating to each goal with relevant final status

Components:
+ 1x Python Node
+ 1x C++ Node
+ 1x .csv File

# Configuring Workspace

## Follow each step carefully

### Workspace Setup and Structure

Cloning this repository will directly install the required workspace `practical_ws`, thus no prior set up is required. This workspace contains the working solution to each task.

The workspace tree has the following structure before building:

practical_ws
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
├── README.md
└── src
    ├── autonomous_nav
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── sim_control.launch
    │   ├── package.xml
    │   ├── README.md
    │   ├── resources
    │   │   └── coordinates.csv
    │   └── src
    │       ├── traverse_goals.cpp
    │       └── traverse_goals.py
    └── simulation_control
        ├── CMakeLists.txt
        ├── launch
        │   └── sim_control.launch
        ├── package.xml
        ├── README.md
        └── src
            ├── circle_driver.cpp
            └── circle_driver.py

### Git Clone

This repository should be cloned in the desired directory, e.g. `~/`, as it contains the entire workspace.

To do this, either method can be used:
```sh
cd ~/
git clone https://github.com/DamianHancock/practical_ws.git
```
Or if you are using SSH keys:
```
cd ~/
git clone git@github.com:DamianHancock/practical_ws.git
```

### Catkin Build

Build and source the workshop, which will successfully build each package contained in this practical test.

```sh
cd ~/practical_ws
catkin build
source devel/setup.bash
```

If the build command fails, install the required tools [here](https://catkin-tools.readthedocs.io/en/latest/installing.html).
