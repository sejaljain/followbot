# Follow Bot
To launch turtlebot and track with __color__ markers in Gazebo:
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To launch turtlebot and track with __shape__ markers in Gazebo:
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```


## Map Specification
- [color.png](worlds/color.png)
	- turtlebot turns left when seeing green marker
	- turtlebot turns right when seeing blue marker
	- turtlebot stops on the red marker
	<p align="center">
	  <img src="imgs/color_map.png">
	</p>

- [shape.png](worlds/shape.png)
	- turtlebot turns left when seeing triangle pointing left
	- turtlebot turns right when seeing triangle pointing right
	- turtlebot stops on the star marker
	<p align="center">
	  <img src="imgs/shape_map.png">
	</p>
