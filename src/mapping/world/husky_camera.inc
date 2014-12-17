
# Desc: Device definitions for UDM's husky
# Author: Cheng-Lung Lee
# Date: 29 Apr 2007
# Updated: 23 April 2010 by Alex Szmatula

# husky uses the 270 ladar
#include "sick.inc"
#include "igvc.inc"



define laser_body model
(
size [ .1 .1 .1] 
color "blue"
)


define topurg ranger
(
	sensor( 			
    range [ 0.0  30.0 ]
    fov 270.25
   samples 1081
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)


define husky_sicklaser270 ranger
(
  color "blue"
  size [ 0.10 0.10 0.1 ]
sensor( 			
    range [ 0.0  30.0 ]
    fov 270.25
   samples 1081
  )
#laser_body (pose[0 0 0 0])
  # generic model properties
)

define husky_laser husky_sicklaser270
(
  color "grey50"
  size [ 0.10 0.10 0.1 ]
#husky_sicklaser270 (pose [0 0 0 0])
)

define husky_camera camera
 (
    # laser properties
    pose [0 0 1.25 0]
    # Stingray F080C
    #resolution [ 1024 768 ]
    # max resolution stage can offer 745x418-> 4:3 418/3*4=557
    resolution [ 557 418 ]
    range [ 0.2 16.0 ]
    # FOV from camera specs for 1/3" sensor @4:3
    # http://www.theiatech.com/prodspecs.php?lens=MY110
    fov     [ 110.0 94.0 ]
    pantilt [ 0.0   35.0 ]

    # model properties
    size [ 0.1 0.05 0.05 ]
    color "red"
    watts 100.0 # TODO find watts for sony pan-tilt camera
 )

define husky_wheel model
(
size [ .3 .1 .3] 
color "black"
)
define husky_body model
(
size [ 1 .6 0.3] 
color "yellow"
)
define husky_dgps model
(
size [ .06 .06 .02] 
color "white" 
)

define husky_lidar model
(
size [ .1 .1 .02] 
color "grey50" 
)

define husky_post model
(
size [ 0.02 0.02 1.3] 
color "black" 
)
define husky_post_top model
(
size [ 0.02 0.5 0.02] 
color "black" 
)

# a husky
define husky_base position
(
  # watchdog_timeout .3 # 300ms watchdog
  # actual size in meters
  size [ 1.0 0.6 0.001]
  color "yellow"
  # the center of rotation is offset from its center of area
  # Center of Area = (0.2584 , 0.000)
  origin [ 0 0 0 0]

  gui_nose 0 # don't need it since we have the SICK on the front
  gui_outline 1
  # estimated mass in KG
  mass 114.0

  husky_body ( pose [  0 0 0.1 0] )
  husky_wheel ( pose [  .25 -.35 0 0] )
  husky_wheel ( pose [  .25  .35 0 0] )
  husky_wheel ( pose [ -.25 -.35 0 0] )
  husky_wheel ( pose [ -.25  .35 0 0] )
# Draw GPS (Center of rotation)
  husky_dgps    ( pose [0 0 1.72 0] )
  husky_post    (pose [  0 -.25 0.4 0] )
  husky_post    (pose [  0  .25 0.4 0] )
  husky_post_top(pose [  0  0 1.7 0] )
husky_camera(pose [  0  0 1.65 0] )
  # Draw LADAR
  husky_lidar ( pose [ 0.45 0 0.4 0] ) # dummy lidar for display
  husky_laser ( pose [ 0.45 0 0.15 0] ) # not showing on stage camera

  # differential steering model
  drive "diff"

  localization "gps"
  localization_origin [ 0 0 0 ]
)  

define husky husky_base
(  
    # Body shape:
  block( 
#    points 8
#    point[0] [-0.2 0.12]
#    point[1] [-0.2 -0.12]
#    point[2] [-0.12 -0.2555]
#    point[3] [0.12 -0.2555]
#    point[4] [0.2 -0.12]
#    point[5] [0.2 0.12]
#    point[6] [0.12 0.2555]
#    point[7] [-0.12 0.2555]
#    z [0 1.22]
    points 6
    point[0] [  -0.48    0.310  ]  # East bar by wheel
    point[1] [  -0.1752  0.310  ]  # Northern point
    point[2] [  0.48     0.120  ]  # North of west point
    point[3] [  0.48    -0.120  ]  # South of west point
    point[4] [  -0.1752 -0.310  ]  # Southern Point
    point[5] [  -0.48   -0.310  ]  # South east point
    z [0 0.46]

    )
    
)

 
