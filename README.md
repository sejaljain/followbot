Lab 5
Sejal Jain sj2735
Quinn Torres qmt2002

Usage: 

There are 3 python files that correspond to each part: part1.py, part2.py and part3.py. Instructions on how to replicate videos for each part is shown below. 

Part 1:
In 3 separate terminal windows run the following commands:
1) roscore
2) roslaunch followbot launch.launch
3) python part1.py

Part 2:
In 3 separate terminal windows run the following commands:
1) roscore
2) ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
3) python part3.py

Part 3:
In 3 separate terminal windows run the following commands:
1) roscore
2) ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
3) python part3.py


Method: 
For part 1, the autonomous followbot code provided in class was used with no modifications. For part 2, 4 masks were made for each color. The yellow upper and lower bounds had to be slightly edited to account for the other colors needed. Moments for each color were also calculated and when the moment was nonzero for green, blue, or red, appropriate actions are made. Instead of stopping at the green and blue turn checkpoints and turning 90 degrees in one go, our bot gradually makes the turn as it crosses the green and blue markers making for a smoother transition. For part 3, two masks are used for the red markers and for the yellow path. To make the turns, the moment of the red mask is calculated. A variable called r_err is calculated which is the amount the x-moment of the red mask is offset from the center of the image. The turn speed is a function of the variable r_err. As in part 2, the turn happens gradually as the robot approaches and crosses the red arrows instead of stopping at one marker and making the full 90 degree turn. 

[WE NEED TO ADD SOMETHING ABOUT HOW WE STOP THE ROBOT]


Video: a link to the Youtube video of working demos. You should concatenate demo videos of all parts into one single video.


