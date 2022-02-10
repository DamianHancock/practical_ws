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

## Workspace Setup

Make sure you are using the Catkin Workspace `/practical_ws` and that was previously cloned from [here](https://github.com/DamianHancock/practical_ws).
Don't forget to build and source!

The Husky Visualisation and Navigation Packages have been included in this repository so no further install is necessary.

## Launching the Navigation Stack

Follow each step below:
```
cd ~/practical_ws
catkin build
source devel/setup.bash
```

The launch sequence includes `gazebo`, `rviz`, `move_base` and `task_two_node` in a single terminal window:

C++ Node:
```bash
roslaunch autonomous_nav task_two_cpp.launch
```
Python Node:
```bash
roslaunch autonomous_nav task_two_py.launch
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
