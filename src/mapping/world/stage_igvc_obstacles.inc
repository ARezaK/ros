# stageros IGVC obstacles
# stage_IGVC_obstacles.inc

define box model
(
  size [0.5 0.5 0.5]
  gui_nose 0
)


define circle model
(
#   block (
#    points 16                          
#    point[0]     [0.500000 0.000000 ]   
#    point[1]     [0.461940 0.191341 ]   
#    point[2]     [0.353553 0.353553 ]   
#    point[3]     [0.191341 0.461940 ]   
#    point[4]     [-0.000000 0.500000 ]  
#    point[5]     [-0.191341 0.461940 ]  
#    point[6]     [-0.353553 0.353553 ]  
#    point[7]     [-0.461940 0.191341 ]  
#    point[8]     [-0.500000 -0.000000 ] 
#    point[9]     [-0.461940 -0.191341 ] 
#    point[10]    [-0.353553 -0.353553 ]
#    point[11]    [-0.191341 -0.461940 ]
#    point[12]    [0.000000 -0.500000 ] 
#    point[13]    [0.191341 -0.461940 ] 
#    point[14]    [0.353553 -0.353553 ] 
#    point[15]    [0.461940 -0.191341 ] 
#   )
   block (
	points 8

	point[0]	[0.500000 0.000000]
	point[1]	[0.353553 0.353553]
	point[2]	[-0.000000 0.500000]
	point[3]	[-0.353553 0.353553]
	point[4]	[-0.500000 -0.000000]
	point[5]	[-0.353553 -0.353553]
	point[6]	[0.000000 -0.500000]
	point[7]	[0.353553 -0.353553]
	z [0 1]
   )

   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams
#  blobfinder_return 1 # Seen by blobfinders  
   fiducial_return 1   # Seen as "1" fiducial finders


   
# Matlab code to generate circle polygon
#     numpoints = 16;
#    angle = double([0:360/numpoints:360]);
#    X = cosd(angle)/2;
#    Y = sind(angle)/2;
#    fprintf('\n\n\tpolygon[0].points %d\n', numpoints);
#    for i = 0:numpoints-1
#         fprintf('\tpolygon[0].point[%d]\t[%f %f]\n', i, X(i+1), Y(i+1))
#    end
#



)






# Matlab code to generate boxed circle for barrel
#     numpoints = 8;
#    angle = double([0:360/numpoints:360]);
#    X = cosd(angle)/2;
#    Y = sind(angle)/2;
#    fprintf('\n\n\tpolygon[0].points #d\n', numpoints);
#    for i = 0:numpoints-1
#         fprintf('\tpolygon[0].point[#d]\t[#f #f]\n', i, X(i+1), Y(i+1))
#    end
# 
#    plot(X,Y)
#    
#    
#    D_barrel=0.60
#    d=0.01
#    
#    
#    x2=[X X(1)]*(D_barrel-d)
#    y2=[Y Y(1)]*(D_barrel-d)
#    w=((x2(1)-x2(2))^2+(y2(1)-y2(2))^2)^0.5
# 
#    for i=1:length(X)
#        x3(i)=(x2(i)+x2(i+1))/2
#        y3(i)=(y2(i)+y2(i+1))/2
#    end
#    
#       for i = 0:numpoints-1
#         fprintf('\tbox ( pose [#f #f 0 #f ] size [#f #f 1.0] color "orange")\n',  x3(i+1), y3(i+1),i*(360/numpoints)+(360/numpoints/2),d,w )
#    end








define barrel model
(
   size [0.1 0.1 0.001]  # 1 meter tall, .61 meter diameter base
#   circle ( pose [ 0 0 0 0] size [0.55 0.55 .2] color "orange")
#   circle ( pose [ 0 0 .2 0] size [0.51 0.51 .2] color "white")
#   circle ( pose [ 0 0 .4 0] size [0.49 0.49 .2] color "orange")
#   circle ( pose [ 0 0 .6 0] size [0.47 0.47 .2] color "white")
#   circle ( pose [ 0 0 .8 0] size [0.45 0.45 .2] color "orange")
   circle ( pose [ 0 0 0  0] size [0.61 0.61 .2] color "orange")
   circle ( pose [ 0 0 .2 0] size [0.61 0.61 .2] color "white")
   circle ( pose [ 0 0 .4 0] size [0.61 0.61 .2] color "orange")
   circle ( pose [ 0 0 .6 0] size [0.61 0.61 .2] color "white")
   circle ( pose [ 0 0 .8 0] size [0.61 0.61 .2] color "orange")

   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams
  gripper_return 1
  gui_move 1
  gui_nose 0
  fiducial_return 1



	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "orange")

)


define barrel_blue model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "blue")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
  
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "blue")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "blue")

)

define barrel_red model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "red")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "red")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "red")
 
)
define barrel_yellow model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "yellow")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "yellow")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "yellow")

)

define barrel_black model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "black")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "black")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "black")

)


define barrel_orange model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "orange")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "orange")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "orange")

)

define barrel_white model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.61 0.61 1] color "white")

   gripper_return 1
   obstacle_return 1 # Can hit things.
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0
	box ( pose [0.251798 0.104298 0 22.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [0.104298 0.251798 0 67.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [-0.104298 0.251798 0 112.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [-0.251798 0.104298 0 157.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [-0.251798 -0.104298 0 202.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [-0.104298 -0.251798 0 247.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [0.104298 -0.251798 0 292.500000 ] size [0.010000 0.225783 1.0] color "white")
	box ( pose [0.251798 -0.104298 0 337.500000 ] size [0.010000 0.225783 1.0] color "white")

)



# TODO check the real size of the post and update it
define post_red model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
# myblock        ( pose [ 0 0 0 0.000000] size [0.1 0.1 1] color "red" )
   circle ( pose [ 0 0 0 0] size [0.1 0.1 1] color "red")

   gripper_return 1
   obstacle_return 1 # flag is not an obstacle
   ranger_return 1   # reflects sonar beams

  gui_move 1
  gui_nose 0
  fiducial_return 0

	box ( pose [0.038410 0.015910 0 22.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [0.015910 0.038410 0 67.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [-0.015910 0.038410 0 112.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [-0.038410 0.015910 0 157.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [-0.038410 -0.015910 0 202.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [-0.015910 -0.038410 0 247.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [0.015910 -0.038410 0 292.500000 ] size [0.010000 0.034442 1.0] color "red")
	box ( pose [0.038410 -0.015910 0 337.500000 ] size [0.010000 0.034442 1.0] color "red")

)

define post_blue model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.1 0.1 1] color "blue")

   gripper_return 1
   obstacle_return 1 # flag is not an obstacle
   ranger_return 1   # reflects sonar beams

   gui_move 1
   gui_nose 0
   fiducial_return 0
   
	box ( pose [0.038410 0.015910 0 22.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [0.015910 0.038410 0 67.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [-0.015910 0.038410 0 112.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [-0.038410 0.015910 0 157.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [-0.038410 -0.015910 0 202.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [-0.015910 -0.038410 0 247.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [0.015910 -0.038410 0 292.500000 ] size [0.010000 0.034442 1.0] color "blue")
	box ( pose [0.038410 -0.015910 0 337.500000 ] size [0.010000 0.034442 1.0] color "blue")
   
)
define post_yellow model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.1 0.1 1] color "yellow")

   gripper_return 1
   obstacle_return 1 # flag is not an obstacle
   ranger_return 1   # reflects sonar beams

   gui_move 1
   gui_nose 0
   fiducial_return 0
	box ( pose [0.038410 0.015910 0 22.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [0.015910 0.038410 0 67.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [-0.015910 0.038410 0 112.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [-0.038410 0.015910 0 157.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [-0.038410 -0.015910 0 202.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [-0.015910 -0.038410 0 247.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [0.015910 -0.038410 0 292.500000 ] size [0.010000 0.034442 1.0] color "yellow")
	box ( pose [0.038410 -0.015910 0 337.500000 ] size [0.010000 0.034442 1.0] color "yellow")

)
define post_white model
(
   size [0.1 0.1 .001]  # 2 meter diameter base
   
   circle ( pose [ 0 0 0 0] size [0.1 0.1 1] color "white")

   gripper_return 1
   obstacle_return 1 # flag is not an obstacle
   ranger_return 1   # reflects sonar beams

   gui_move 1
   gui_nose 0
   fiducial_return 0

	box ( pose [0.038410 0.015910 0 22.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [0.015910 0.038410 0 67.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [-0.015910 0.038410 0 112.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [-0.038410 0.015910 0 157.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [-0.038410 -0.015910 0 202.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [-0.015910 -0.038410 0 247.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [0.015910 -0.038410 0 292.500000 ] size [0.010000 0.034442 1.0] color "white")
	box ( pose [0.038410 -0.015910 0 337.500000 ] size [0.010000 0.034442 1.0] color "white")

)

define Barricade_Type2_top_orange model
(
size [ .6 .1 .2] 
color "orange"
)

define Barricade_Type2_top_white model
(
size [ .6 .1 .2] 
color "white"
)

define Barricade_Type1 model
(
	size [0.1 .1 0.001]
	myblock (size [.1 .1 1] pose [-0.25 0 0 0] color "white")
	myblock (size [.1 .1 1] pose [0.25 0 0 0] color "white")
	Barricade_Type2_top_orange ( pose [0 0 0.9 0] )
	Barricade_Type2_top_white ( pose [0 0 0.3 0] )
	gripper_return 1
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)
define Barricade_Type2 model
(
	size [0.1 .1 0.001]
	myblock (size [.1 .1 1] pose [-0.25 0 0 0] color "white")
	myblock (size [.1 .1 1] pose [0.25 0 0 0] color "white")
	Barricade_Type2_top_orange ( pose [0 0 0.9 0] )
	Barricade_Type2_top_orange ( pose [0 0 0.3 0] )
	gripper_return 1
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)

define Fance2m model
(
	size [0.2 1 0.001]
	myblock (size [.4 2 1] pose [0 0 0 0] color "gray")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)
define Fance3m model
(
	size [0.2 1 0.001]
	myblock (size [.2 3 1] pose [0 0 0 0] color "RosyBrown")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)

define Fance5m model
(
	size [0.2 1 0.001]
	myblock (size [.2 5 1] pose [0 0 0 0] color "RosyBrown")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)
define Fance10m model
(
	size [0.2 5 0.001]
	myblock (size [.2 10 1] pose [0 0 0 0] color "RosyBrown")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)
define Fance15m model
(
	size [0.2 10 0.001]
	myblock (size [.2 15 1] pose [0 0 0 0] color "RosyBrown")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)
define Fance20m model
(
	size [0.2 10 0.001]
	myblock (size [.2 20 1] pose [0 0 0 0] color "RosyBrown")
	
	gripper_return 0
    obstacle_return 1 # flag is not an obstacle
    ranger_return 1   # reflects sonar beams
    gui_move 1
    gui_nose 0
    fiducial_return 0
)

