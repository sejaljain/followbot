# Lab 5 - Follow Bot
Lab 5 for [COMSW4733 Computational Aspects of Robotics](http://www.cs.columbia.edu/~allen/F18/index.html) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Introduction
In this lab, you are required to make the turtlebot follow a yellow track and action differently at intersections based on visual information. This is simulated in a GazeBo simulator.

## Usage
This ROS package allows you to load 4 different maps in GazeBo.

### Prerequisites
The package is tested on `python 2.7`, `ROS Indigo`, `Ubuntu 14.04` with `OpenCV 3.1.0` and `numpy 1.15.1`.

### Commands
To launch turtlebot and map for part 1
```
roslaunch followbot launch.launch
```

To launch turtlebot and map for part 2
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To launch turtlebot and map for part 3
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```

To launch the map for extra credits:
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world
```
## Instructions and Rubric
- Part 1: Preparation (**10 points**)
 [color.png](worlds/color.png)
	- turtlebot turns left when seeing green marker
	- turtlebot turns right when seeing blue marker
	- turtlebot stops on the red marker
	<p align="center">
	  <img src="imgs/color_map.png", height="450">
	</p>

- Part 2: Map with color markers
 [shape.png](worlds/shape.png)
	- turtlebot turns left when seeing triangle pointing left
	- turtlebot turns right when seeing triangle pointing right
	- turtlebot stops on the star marker
	<p align="center">
	  <img src="imgs/shape_map.png", height="450">
	</p>

	- Part 3: Map with color markers
	 [shape.png](worlds/shape.png)
		- turtlebot turns left when seeing triangle pointing left
		- turtlebot turns right when seeing triangle pointing right
		- turtlebot stops on the star marker
		<p align="center">
		  <img src="imgs/shape_map.png", height="450">
		</p>
