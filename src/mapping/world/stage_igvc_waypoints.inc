

define waypoint2m model
(
   size [1.41 1.41 0.001]  # 2 meter diameter base
   circle ( pose [ 0 0 0 0] size [2    2    0.01]  color "yellow" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.25 0.25 0.011] color "white" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.05 0.05 0.012] color "orange" obstacle_return 0)
   #myblock ( pose [ 0 0 .01 0] size [0.25 0.25 .011] color "white" obstacle_return 0 )
   #myblock ( pose [ 0 0 .02 0] size [0.05 0.05 .012] color "orange" obstacle_return 0 )

   gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 0 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)

define waypoint2m_movable model
(
   size [1.41 1.41 0.001]  # 2 meter diameter base
   circle ( pose [ 0 0 0 0] size [2    2    0.01]  color "yellow" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.25 0.25 0.011] color "white" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.05 0.05 0.012] color "orange" obstacle_return 0)
   #myblock ( pose [ 0 0 .01 0] size [0.25 0.25 .011] color "white" obstacle_return 0 )
   #myblock ( pose [ 0 0 .02 0] size [0.05 0.05 .012] color "orange" obstacle_return 0 )

   gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 1 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)
define waypoint1m model
(
   size [0.7 0.7 0.001]  # 2 meter diameter base
   circle ( pose [ 0 0 0 0] size [1    1    0.01]  color "yellow" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.25 0.25 0.011] color "white" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.05 0.05 0.012] color "orange" obstacle_return 0)
   #myblock ( pose [ 0 0 .01 0] size [0.25 0.25 .011] color "white" obstacle_return 0 )
   #myblock ( pose [ 0 0 .02 0] size [0.05 0.05 .012] color "orange" obstacle_return 0 )

   gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 0 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)
define waypoint1m_movable model
(
   size [0.7 0.7 0.001]  # 2 meter diameter base
   circle ( pose [ 0 0 0 0] size [1    1    0.01]  color "yellow" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.25 0.25 0.011] color "white" obstacle_return 0)
   circle ( pose [ 0 0 0 0] size [0.05 0.05 0.012] color "orange" obstacle_return 0)
   #myblock ( pose [ 0 0 .01 0] size [0.25 0.25 .011] color "white" obstacle_return 0 )
   #myblock ( pose [ 0 0 .02 0] size [0.05 0.05 .012] color "orange" obstacle_return 0 )

   gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 1 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)


define checker_board model
(
    size [ 2 2 0.001]
    myblock ( pose [ 0     0   0 0] size   [2 2 .003] color "white" obstacle_return 0 )
    myblock ( pose [ 0.5   0.5 0 0] size   [1 1 .004] color "black" obstacle_return 0 )
    myblock ( pose [ -0.5 -0.5 0 0] size   [1 1 .004] color "black" obstacle_return 0 )
       gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 0 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)

define checker_board_v model
(
    size [ 0.01 0.01 0.01]
    myblock ( pose [ 0   0   0    0] size   [1.2 .003 2  ] color "white" obstacle_return 0 )
    myblock ( pose [ 0   0   0.1  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0   0   0.5  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0   0   0.9  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0   0   1.3  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0   0   1.7  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )

    myblock ( pose [ 0.4   0   0.1  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.4   0   0.5  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.4   0   0.9  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.4   0   1.3  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.4   0   1.7  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )

    myblock ( pose [ -0.4   0   0.1  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.4   0   0.5  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.4   0   0.9  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.4   0   1.3  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.4   0   1.7  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )

    myblock ( pose [ 0.2   0   0.3  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.2   0   0.7  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.2   0   1.1  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ 0.2   0   1.5  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )

    myblock ( pose [ -0.2   0   0.3  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.2   0   0.7  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.2   0   1.1  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )
    myblock ( pose [ -0.2   0   1.5  0] size   [0.2 .004 0.2] color "black" obstacle_return 0 )

   gripper_return 0
   obstacle_return 0 # set to 0 so waypoint is not an obstacle. can't be hit
   ranger_return 0   # set to 0 for no reflects sonar/laser beams
   gui_outline 0
   gui_move 0 # set it to 0 , we don't want waypoint to be moved
   gui_nose 0
   fiducial_return 1

)



# TODO check the real size of the flag and update it 
define flag_top_red model
(
size [ .2 .01 .15] 
color "red"
obstacle_return 0
)
define flag_top_blue model
(
size [ .2 .01 .15] 
color "blue"
obstacle_return 0
)



define flag_red model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
	myblock        ( pose [ 0 0 0 0.000000] size [0.010000 0.01 0.3] color "white" obstacle_return 0 )
	flag_top_red ( pose [ 0.1 0 0.3 0.000000] )

   gripper_return 1
   obstacle_return 0 # flag is not an obstacle
   ranger_return 0   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0

)

define flag_blue model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
	myblock        ( pose [ 0 0. 0 0.000000] size [0.010000 0.01 0.3] color "white" obstacle_return 0 )
	flag_top_blue ( pose [ 0.1 0.0 0.3 0.000000] )

   gripper_return 1
   obstacle_return 0 # flag is not an obstacle
   ranger_return 0   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0

)



