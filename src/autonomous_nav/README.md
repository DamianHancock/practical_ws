# Autonomous Navigation

**Goal:** Simulated robot navigating smoothly to user-selected waypoints that are loaded from a .csv file

**Overview:** 
This task uses the ROS packages Clearpath makes available for the Husky. 
This includes the Gazebo, RViz and Navigation packages.

## Background

### Navigation Stack
* The navigation is integrated using a 2D LiDAR sensor that can be enabled before launching the Husky in the simulated environment. The LMS1xx sensor from SICK is mounted on the top of the Husky and enables the generation of the global and local costmaps. 
* The global costmap considers the entire known surroundings, while the local generates the object detection services. The Husky is then able to plan it's journey to different coordinates using these costmaps. 
* AMCL (adaptive Monte Carlo localization is used for localising the odemetry of the Husky against the known map using a particle filter. Move base then acts as the controller between the information received from the sensor and localisation data to generate paths. 
* The goals themselves are loaded from a .csv located in `/practical/src/autonomous_nav/resources/coordinates.csv`. These goals are set within the code itself in the current build state, but can also be altered directly in the .csv which will be discussed in the testing procedure.

## Testing and Running

### Workspace Setup

Make sure you are using the Catkin Workspace `/practical_ws` and that was previously cloned from [here](https://github.com/DamianHancock/practical_ws).
Don't forget to build and source!

The Husky Visualisation and Navigation Packages have been included in this repository so no further install is necessary.

### Setting and Saving Goals

* To set custom coordinates before operation the code within `src/nav_util.py` or `src/nav_util.cpp` can be edited. It has been designed this way to avoid overlap when running the code multiple times as it saves the operator manually opening the csv and removing the sections that save at the end of operation
* The `set_goals()` function in `src/nav_util.cpp` is shown below:
![set_goals](https://user-images.githubusercontent.com/64782797/153536779-6de9c01c-4da2-4741-b06d-a11bce90a3ff.png)

* The .csv file lies in the `/resources/` directory and has the following format:
![ss](https://user-images.githubusercontent.com/64782797/153536063-e2622127-b258-4b72-930f-760346f77b66.png)
* It can be seen in this example that the first 3 rows of the .csv were taken as the input goal coordinates, and the last 3 rows show the output of the planned movement
* The status message is saved for extra information directly from `move_base/feedback`
* The time taken (seconds) to reach each goal from the initial position is recorded for additional information

### Launching the Navigation Stack

Follow each step below:
```
$ cd ~/practical_ws
$ catkin build
$ source devel/setup.bash
```

The launch sequence includes `gazebo`, `rviz`, `move_base` and `task_two_node` in a single terminal window:

C++ Node:
```bash
$ export HUSKY_LMS1XX_ENABLED=1
$ roslaunch autonomous_nav task_two_cpp.launch
```
Python Node:
```bash
$ export HUSKY_LMS1XX_ENABLED=1
$ roslaunch autonomous_nav task_two_py.launch
```
* If `rviz` appears cluttered, feel free to turn off the Sensing group of visualisers in the Displays panel

An example of the .launch file is as follows:
```XML
<?xml version="1.0"?>
  <launch>
      <include file="$(find timed_roslaunch)/launch/timed_task_two.launch">
      </include>

      <node pkg="autonomous_nav" name="traverse_goals" type="traverse_goals" output="screen">
      </node>
  </launch>
```

### Expected Operation
* When the launch file has been run Gazebo will first open after a couple of seconds. The Husky Playpen world has been chosen as the basic testing grounds for the autonomous navigation purposes
* RViz will then launch showing appropriate visualisation of sensors
* Move_Base will begin with AMCL configuration loaded
* Then either the C++ or Python node will start based on which launch file was run
* Messages will be displayed when new goals are read from the csv and loaded into the path plan within move_base
* After all goals have attempted to be reached by the Husky, the final coordinates will all be appended to the .csv file for review

