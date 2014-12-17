
# Desc: Device definitions for UDM's Cerberus
# Author: Kevin Barry
# Date: 29 Apr 2007
# Updated: 23 April 2010 by Alex Szmatula

# cerberus uses the 270 ladar
#include "sick.inc"
#include "igvc.inc"

define sicklaser270 ranger
(
	sensor( 			
    range [ 0.0  30.0 ]
    fov 270.25
   samples 1081
  )

  # generic model properties
  color "blue"
  size [ 0.10 0.10 0.10 ]
  myblock ( pose [0 0 0 0] size [ .1 .1 .1] color "blue" )
)

define wheel model
(
size [ .41 .07 .41] 
color "black"
)

# a cerberus
define cerberus_base position
(
  # watchdog_timeout .3 # 300ms watchdog
  # actual size in meters
  # size [ .96 .62 .46]
  size [ .96 .62 .46]

  # the center of rotation is offset from its center of area
  # Center of Area = (0.2584 , 0.000)
  origin [ 0.2584 0.0 0 0]

  gui_nose 0 # don't need it since we have the SICK on the front
  gui_outline 1
  # estimated mass in KG
  mass 114.0

#  block(	
#  # this polygon approximates the shape of a cerberus
#  points 6
#  #     ___.
#  #    /   |
#  #   |    |
#  #    \___|
#
#  # Coordinates start at bottom left and move ccw
#  point[0] [  -0.1524  0.310  ]  # East bar by wheel
#  point[1] [   0.1524  0.310  ]  # Northern point
#  point[2] [   0.8076  0.120  ]  # North of west point
#  point[3] [   0.8076 -0.120  ]  # South of west point
#  point[4] [   0.1524 -0.310  ]  # Southern Point
#  point[5] [  -0.1524 -0.310  ]  # South east point
#  z [0.12 0.46]
#  )

  # Draw wheels - wheel sizes listed were taken from captop
#  rectangle ( size [ .41 .07 .41] pose [ .0 -.365 -1 0] color "black" )
#  rectangle ( size [ .41 .07 .41] pose [ .0 +.365 -1 0] color "black" )
  wheel ( pose [ .0 -.365 -0.46 0] )
  wheel ( pose [ .0  .365 -0.46 0] )

  # Draw GPS (Center of rotation)
  # myblock ( pose [0 0 0 0] size [ .12 .12 .12] color "white" gui_outline 0)


  # Camera/blobfinder
#  ptz ( pose [+.22 0 0] 
#     	fov 180
#        blobfinder(obstacle_return 0)
#        obstacle_return 0
#	)
#
 camera
 (
    # laser properties
    pose [0 0 1.2 0]
    #pose [0 0 0.5 0]
    # Stingray F080C
    #resolution [ 1024 768 ]
    # max resolution stage can offer 745x418
    resolution [ 745 418 ]
    range [ 0.2 16.0 ]
    # FOV from camera specs for 1/3" sensor
    # http://www.theiatech.com/prodspecs.php?lens=MY110
    fov [ 110.0 94.0 ]
    pantilt [ 0.0 35.0 ]
    #pantilt [ 0.0 0.0 ]
    # pantilt [ 0.0 40.0 ]

    # model properties
    size [ 0.1 0.07 0.05 ]
    color "black"
    watts 100.0 # TODO find watts for sony pan-tilt camera
 )
  # Draw LADAR
  sicklaser270 ( pose [ 0.7 0 -0.1 0] )

  # differential steering model
  drive "diff"

  localization "gps"
  localization_origin [ 0 0 0 ]
)  

define cerberus cerberus_base
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

 
