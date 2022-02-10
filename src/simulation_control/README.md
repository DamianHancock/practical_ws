# Simulation Robot Control

**Goal:** Simulated robot navigating in a circular fashion using Daemon or 3D Visulaisation

**Overview:** 
This task uses the ROS packages Clearpath makes available for the Husky. 
This includes the Husky Control, Gazebo and Viz packages.

## Background

### Husky Velocity Messages and Topics

This package takes advantage of the Clearpath Husky and the available sensors onboard, 
including a range of topics such as the velocity of the robots movements in different directions.

The husky contains the `/husky_velocity_controller/cmd_vel` namespace topic published
by `/twist_mux` node and subscribed by `gazebo` node. 
`geometry_msgs/Twist` messages are published to the gazebo node to make the husky move using external input from the nodes produced by the developer.

This can essentially be manually accomplished from command line using:

```sh
rostopic pub -r 10 /husky_velocity_controller/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 2.0,z: 0.0}}'
```

The `circle_driver.cpp` and `circle_driver.py` files publish appropriate
messages to `/husky_velocity_controller/cmd_vel` topic.
These ROS nodes command the simulated Husky to move in a circular path.

## Workspace Setup

Make sure you are using the Catkin Workspace `/practical_ws` and that was previously cloned from [here](https://github.com/DamianHancock/practical_ws).
Don't forget to build and source!

## Linux Service (Daemon)

### Testing Procedure

## 3D Modelling and Simulation

### Testing Procedure

The robot can also be visualised in both Gazebo and RViz.

#### Launch the robot simulation

For launching the simulation of the circle driver code:
* C++ node: 
```bash
$ roslaunch simulation_control task_one_cpp.launch
```
* Python node: 
```bash
$ roslaunch simulation_control task_one_py.launch
```

An example of the .launch file is as follows:

```XML
<?xml version="1.0"?>
    <launch>
        <include file="$(find timed_roslaunch)/launch/timed_task_one.launch">
        </include>
        
        <include file="$(find timed_roslaunch)/launch/timed_roslaunch.launch">
            <arg name="time" value="8" />
            <arg name="pkg" value="simulation_control" />
            <arg name="file" value="circle_driver_py.launch" />
        </include>
    </launch>
```
