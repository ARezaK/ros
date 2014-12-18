ros
===

Ros mapping



This is my catkin_ws folder so technically you dont need everything it.


To run mapping, take the src folder in here and place it in your own catkin_ws.


Do:

catkin_make

source devel/setup.bash


Programs that need (or should be) to be running before using mapping.

**1) Roscore**

roscore

**2) Rviz**

rosrun rviz rviz

Config is included in the mapping folder


**3) Stage**

CD to your world folder (There is one included in the mapping folder)

rosrun stage_ros stageros IGVC2013Auto-Nav_Basic.world


**4) Gmapping or hectormapping**

Within the mapping folder there is a launch file for both gmapping or hectormapping

roslaunch gmapping.launch

OR

rosluanch hector_mapping.launch



**5) Optional - Generating fake lane lines**

Make sure the lane lines are black in your world if you need to generate fake lane lines

afl = ameer's fake lane lines

rosrun mapping afl

This outputs two image topics.

The convertimage topic is the binary image used by mapping

The visualizedimage topic is a visualization of the image before it is converted into binary.


**6) Mapping Program**

rosrun mapping mapping


To see what the program is publishing look at the bottom of the mapping code.
