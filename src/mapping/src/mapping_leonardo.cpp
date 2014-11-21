//********************HEADERS NECESSARY TO USE THIS CODE (ALL ARE INCLUDED WITH THE ROS STANDARD PACKAGES)*****************************
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"                        //To command movements to the robot
#include "geometry_msgs/PoseWithCovariance.h"           //To receive pose messages 
#include "geometry_msgs/PoseWithCovarianceStamped.h"    //To receive pose messages (different format than above)
#include "sensor_msgs/LaserScan.h"                      //To receive messages from Lidar
#include "sensor_msgs/Image.h"                          //To receive messages from camera
#include "geometry_msgs/Polygon.h"                      //Not used 
//#include "geometry_msgs/point.h" //???                //Not used but useful to send short data as points 
#include "nav_msgs/OccupancyGrid.h"                     //To send messages in Occupancy grid map formats
#include "nav_msgs/GridCells.h"                         //Not used
#include "nav_msgs/Odometry.h" //Para obtener la pose   //To receive the pose messages (the one currently used)
#include <math.h>                                       //Library to perform math operations
#include <stdlib.h>                                     
#include <iostream>
#include <fstream>
#include <LinearMath/btQuaternion.h>                    // Library needed to convert  quaterions
#include <LinearMath/btMatrix3x3.h>                     //  into Euler angles
#include <stdio.h>                                      //not used
#include <opencv2/opencv.hpp>                           //not used
#include "opencv2/core/core.hpp"                        //not used
#include <opencv2/highgui/highgui.hpp>                  //not used
#include <opencv/cv.h>                                  //not used
#include <the_mapping/ForGoal.h>                        // Library created to send pose messages of goals
#include "opencv/ml.h"                                  //not used
#include "opencv/cxcore.h"                              //not used
//#include "opencv/types_c.h"                           //not used
//#include "opencv2/cxtypes.h"                          //not used
#include "opencv/highgui.h"                             //not used
//********************************************************************************************************************************************

using namespace std;                        //Don't remember exactly why I included these 
using namespace cv;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i)) //This function makes indexing easier. The output will be the index number of a vector corresponding to
                                             // the element (i,j) in a matrix. sx=number of columns of the map/image. 
                                             // i=element i of the matrix
                                             // j=element j of the matrix     





//*****************************************VARIABLES USED***********************************************************************************
double pose_xinit, pose_yinit, pose_ainit;              //intitial position of the robot values in UTM //(used in alternate pose callback)
double pose_x, pose_y, pose_a;                          //localization input pose values in UTM
bool init_flag = 0;                                     //This is a flag used to know if the initial values are receive yet 
                                                        //(used in alternate pose callback) 
int iy2, ix2;                                           //
double robotx=0, roboty=0, heading=0, pitch=0, roll=0;  //robotx and roboty are the current position of the robot in meters in global coordinates
float map_grid [1000000];                               //map grid is a map in vector format storing the probability values of obstacles 
                                                        //the size is 1000000 because it can be a map of 1000*1000 (is adjustable)
double map_prob_left[1000000];                          //a map storing all the probabilities of a lane line to be part of the left line
double map_prob_right[1000000];                         //a map storing all the probabilities of a lane line to be part of the right linelanes
double robotxant [2];                                   //a vector storing the last 2 positions of the robot (x coordinates) 
double robotyant [2];                                   //a vector storing the last 2 positions of the robot (y coordinates)
double headingant [2];                                  //a vector storing the last 2 positions of the robot (heding in degrees)
double dleftx[40];                                      //last 40 cells classified as left (x coordinates)
double dlefty[40];                                      //last 40 cells classified as left (y coordinates)
double drightx[40];                                     //last 40 cells classified as right (x coordinates
double drighty[40];                                     //last 40 cells classified as right (y coordinates
double gylant, gxlant, gxrant, gyrant, mapxllindant, mapyllindant, angleftant=0, angrightant=0, firstangleleft=0, firstangleright=0;
float res=0.020;		                                //Resolution of lane line images meters/px
//float xof=0.25;			    //Robot offset from images (its actually zero, so is not used)
unsigned int seq=0;		                //sequence for lanelines goal (not used)
unsigned int seq2=0;		            //sequence for gps tail (not used)
unsigned int firstL=0, firstR=0;                        // flags to know if first regions Left and right are already classified/identified
nav_msgs::OccupancyGrid mapa;                           // General Map ...this is "THA MAP"! (includes all the features: obstacles, lane lines, positions...)
nav_msgs::OccupancyGrid mapa2;                          //Obstacles map ...TRESHOLDED
nav_msgs::OccupancyGrid mapa3;                          //Lane lines map linelanes
nav_msgs::OccupancyGrid mapa4;                          //Map of left lanes
nav_msgs::OccupancyGrid mapa5;                          //Map of right lanes
nav_msgs::OccupancyGrid mapa6;                          //Positions map
nav_msgs::OccupancyGrid mapa7;                          //Predicted left
nav_msgs::OccupancyGrid mapa8;                          //Predicted right
nav_msgs::OccupancyGrid maparegiones;               //Map of regions to classify left or right lanes (negative for left, positive for right)
sensor_msgs::LaserScan localmap1;                   //Local map in LIDAR format
sensor_msgs::LaserScan localmap2;                   //Local map in LIDAR format used for flags it seems
geometry_msgs::Polygon forgoal_left;                //not used
geometry_msgs::Polygon forgoal_right;               //not used
geometry_msgs::Polygon forgoal_gps;                 //not used
geometry_msgs::Point32 leftlane;                    //not used
geometry_msgs::Point32 rightlane;                   //not used
geometry_msgs::Point32 gpstail;                     //not used
//***************************************End of global variables**************************************************************************************


//***********************************************Callback from LIDAR******************************************************************
//Whenever the LIDAR sends a message this callback is called
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	//Types of messages received in this callback:
		//float32 angle_min  msg->angle_min
		//float32 angle_max
		//float32 angle_increment
		//float32 time_increment
		//float32 scan_time
		//float32 range_min
		//float32 range_max
		//float32[] ranges
		//float32[] intensities
			

			//Set of variables:
			unsigned int minIndex = 10;	//Minimum index of ladar to process
			//Maximum index of ladar to process:
			unsigned int maxIndex = floor((msg->angle_max-msg->angle_min)/msg->angle_increment)-10; 
			//float closestRange = msg->ranges[minIndex];
			//int closestRange_index;
            float x_lidar, y_lidar, glob_x, glob_y, xl_lidar, yl_lidar;
			float glob_xl, glob_yl,cos_angle, sin_angle, ldoff, range;
			unsigned int mapxlind, mapylind, mapxind, mapyind; 	
			//localmap.ranges= msg->ranges;
			ldoff=0.7;			//offset distance of lidar from robot baselink in meters

			//Each element of the message received is transformed into local coordinates and then to global coordinates:
			for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
				cos_angle=cos(((currIndex*msg->angle_increment)+ msg->angle_min));
				sin_angle=sin(((currIndex*msg->angle_increment)+ msg->angle_min));
				x_lidar=msg->ranges[currIndex]*cos_angle + ldoff;//Distance x from robot to obstacle
				y_lidar=msg->ranges[currIndex]*sin_angle;//Distance y from robot to obstacle
				glob_x=(x_lidar*cos(heading))-((y_lidar)*sin(heading))+robotx; //Distance x from map origin to obstacle
				glob_y=(x_lidar*sin(heading))+((y_lidar)*cos(heading))+roboty; //Distance y from map origin to obstacle
				
				mapxind=floor(glob_x/mapa.info.resolution);	//Index x to the map
				mapyind=floor(glob_y/mapa.info.resolution);	//Index y to the map
				
				//If the robot is not in the origin, and global coordinates of the obstacle are valid (not negative), and ranges of lidar below max and greater than min and global coordinates in the boundaries of the map, then:
				if ((roboty!=0) && (robotx!=0) && glob_y>0 && glob_x>0 && (msg->ranges[currIndex]<msg->range_max) && (msg->ranges[currIndex]>msg->range_min) && glob_x<mapa.info.width*mapa.info.resolution && glob_y<mapa.info.height*mapa.info.resolution ) {
					    
                        //calculate the probability of the cell to be an obscatle according to the distance from the robot and increase current value in the probability map 
						map_grid[MAP_IDX(mapa.info.width, mapxind, mapyind)]+=20*(-0.000013*pow(msg->ranges[currIndex],3)+0.001593*pow(msg->ranges[currIndex],2)-0.06653*msg->ranges[currIndex]+1.00);
						
						//if reaches a value greater than 105 then save in global map and in obstacles map. Make it 410 just to be sure it will be always greater than 105  
						if (map_grid[MAP_IDX(mapa.info.width, mapxind, mapyind)]>105) {
						mapa2.data[MAP_IDX(mapa2.info.width, mapxind, mapyind)]=100;
						mapa.data[MAP_IDX(mapa2.info.width, mapxind, mapyind)]=100;
						map_grid[MAP_IDX(mapa.info.width, mapxind, mapyind)]=410;
						}
					
				}
				//If there's a 0 in the current index (means that there is no obstacle in that index (or angle)
                //then from 10 mteres to the robot in the same direction, set all the cells as free cells
                
				if (msg->ranges[currIndex]==0){ range=10.0;}
				else {range=msg->ranges[currIndex];}
				//Loop to compute free cells for each beam
                //dislib is the distance (free from obstacles) from the robot to the cell before the obstacle, each cell between the robot and the
				//obstacle is classified as free:
				for (double dislib=mapa.info.resolution; dislib < (range-(sqrt(3)*mapa.info.resolution)); dislib=dislib+mapa.info.resolution)
					{
					xl_lidar=dislib*cos_angle+ldoff;                                    //distance from robot to current cell (x)
					yl_lidar=dislib*sin_angle;                                          //distance from robot to current cell (y)
					glob_xl=(xl_lidar*cos(heading))-(yl_lidar*sin(heading))+robotx;     //distance from origin to current cell (x)
					glob_yl=(xl_lidar*sin(heading))+(yl_lidar*cos(heading))+roboty;     //distance from origin to current cell (y)
					mapxlind=floor(glob_xl/mapa.info.resolution);                       //No. of pixels to x direction
					mapylind=floor(glob_yl/mapa.info.resolution);                       //No. of pixels to y direction
					
					//if robot is not in origin, and distance to current pixels is inside the map boundaries (>0 and <max values) then:
					if ((roboty!=0) && (robotx!=0) && glob_yl>0 && glob_xl>0 && glob_xl<mapa.info.width*mapa.info.resolution && glob_yl<mapa.info.height*mapa.info.resolution ) {
						
						
				
							map_grid[MAP_IDX(mapa.info.width, mapxlind, mapylind)]-=5*(0.000014*pow(dislib,3)-0.000735*pow(dislib,2)-0.01187*dislib+1.00);
							
							if (map_grid[MAP_IDX(mapa.info.width, mapxlind, mapylind)]<-155) {
							mapa2.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]=0;
							map_grid[MAP_IDX(mapa.info.width, mapxlind, mapylind)]=-185;
								if ((mapa.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]==50) || (mapa.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]==100) ) {		//ensures that the cell was unknown before and is not  a laneline or something else
								mapa.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]=0;
								}
							}
						
					}
					
				}
			}
			
	}

//***************************ADDED MAY 26 2013*************************************
//                       alternate pose callback
/*
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//get intitial position
	double roll, pitch, yaw;
	unsigned int mapxind, mapyind;

	// Convert quaternion to Euler using tf package
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
    msg->pose.pose.orientation.w);
    btMatrix3x3(q).getEulerYPR(heading, pitch, roll);
	
	//get pose estimate
	pose_x = msg->pose.pose.position.x; // Read current x position
	pose_y = msg->pose.pose.position.y; // Read current y position
	pose_a = heading; //Read current heading

	//subtract initial position
	 if (init_flag == 0)
		{
		 //get initial use current values as initial reading
		 pose_xinit = pose_x;
		 pose_yinit = pose_y;
		 pose_ainit = pose_a;

		 //mark initial values as read
		 init_flag = 1;
	  	}

	//find dx and dy
	robotx = pose_x-pose_xinit;
	roboty = pose_y-pose_yinit;
	//heading = pose_a-pose_ainit;
    mapxind=floor(robotx/mapa6.info.resolution);
	mapyind=floor(roboty/mapa6.info.resolution);
			if (robotx>0 && roboty>0 && roboty<mapa6.info.height*mapa6.info.resolution && robotx<mapa6.info.width*mapa6.info.resolution) {
				mapa6.data[MAP_IDX(mapa6.info.width, mapxind, mapyind)]=100;	//positions map
				mapa.data[MAP_IDX(mapa6.info.width, mapxind, mapyind)]=85;	//general map
            }

	//test message
	ROS_INFO_STREAM("X: " << robotx << "Y:"<< roboty << "heading:" << heading );
	ROS_INFO_STREAM("X init: " << pose_xinit << "Y init:"<< pose_yinit << "init heading:" << pose_ainit );

	gpstail.x = robotx;
   	gpstail.y = roboty;
	gpstail.z = seq2;
	forgoal_gps.points.push_back(gpstail);
	seq2++;
}*/


 
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		float dx=0;
		float dy=0;
		float dt=0;
		float ang;
		unsigned int index;

//Types of messages:
	    //geometry_msgs/PoseWithCovariance pose
  	      //geometry_msgs/Pose pose
		//geometry_msgs/Point position   
 		//float64 x			
 		// float64 y
 		// float64 z
		//geometry_msgs/Quaternion orientation
 		// float64 x
  		//float64 y
  		//float64 z
  		//float64 w
			int mapxind, mapyind, mapxgind, mapygind;
			double x_right [120];
			double y_right [120];
			double x_left [120];
			double y_left [120];
			double angles_left [120];
			double angles_right [120];
			double n, m, avgx, avgy, avgtot, sumang_x, sumang_y, goal_x, goal_y;
			
			for (unsigned int i = 0; i<120; i++) {
				x_right[i]=0;
				y_right [i];
				x_left [i];
				y_left [i];
				angles_left [i];
				angles_right [i];
			} 

			btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
    			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
    			msg->pose.pose.orientation.w);
    			btMatrix3x3(q).getEulerYPR(heading, pitch, roll);
	
	/*//*****************************UPDATED MAY 26 2013*************************************	
	//added intermediate variables for full UTM pose
			pose_x = msg->pose.pose.position.x;  //Update x position
			pose_y = msg->pose.pose.position.y;  //Update y position
			pose_a = heading;	
	//subtract UTM pose values from given initial UTM pose values to get local position
			robotx = pose_x - pose_xinit;
			roboty = pose_y - pose_yinit;
			heading = heading - pose_ainit;	
			ROS_INFO_STREAM("X: " << robotx << "Y:"<< roboty << "heading:" << heading );
	//*************************************************************************************
	*/		

			for (unsigned int i= 0; i< 1; i++) {
				robotxant[i]=robotxant[i+1];
				robotyant[i]=robotyant[i+1];
				headingant[i]=headingant[i+1];
			}

			robotxant[1]=robotx;
			robotyant[1]=roboty;
			headingant[1]=heading;
			
			robotx = msg->pose.pose.position.x; 
			roboty = msg->pose.pose.position.y;
			ROS_INFO_STREAM("X: " << robotx << "Y:"<< roboty << "heading:" << heading );

			mapxind=floor(robotx/mapa.info.resolution);
			mapyind=floor(roboty/mapa.info.resolution);
			if (robotx>0 && roboty>0 && roboty<mapa6.info.height*mapa6.info.resolution && robotx<mapa6.info.width*mapa6.info.resolution) {
				mapa6.data[MAP_IDX(mapa6.info.width, mapxind, mapyind)]=100;	//positions map
				mapa.data[MAP_IDX(mapa6.info.width, mapxind, mapyind)]=85;	//general map
            		}

			 iy2=(mapyind-100); 	// in map coordinates
			 ix2=(mapxind-100);	//in map coordinates
			//UPDATE LOCAL MAPS

			for (unsigned int lmi = 0; lmi < 1081; lmi++) {
				localmap1.ranges[lmi]=0;
				localmap2.ranges[lmi]=0,
				localmap1.intensities[lmi]=0.5;
				localmap2.intensities[lmi]=0.5;
				//TODO TELL UTAYBA OR JOE THAT A 0 MEANS NOTHING
			}
			ROS_INFO_STREAM("Es el localmap.....?");
			for (int iy=iy2; iy<=(mapyind+100); iy++) {
				for (int ix=ix2; ix<=(mapxind+100); ix++) {
				if (ix>=0 && iy>=0 && ix<=mapa.info.width && iy<=mapa.info.height) {
				/*	if (mapa.data[MAP_IDX(mapa.info.width, ix, iy)]==30) {	//left lane
						dx=(ix-mapxind)*mapa.info.resolution;
						dy=(iy-mapyind)*mapa.info.resolution;
						dt=sqrt((dx*dx)+(dy*dy));
						ang=(atan2(dy,dx))-heading;
						index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
						localmap1.ranges[index]=dt;
						localmap1.intensities[index]=0;		//left lane
					}
					if (mapa.data[MAP_IDX(mapa.info.width, ix, iy)]==60) {	//right lane
						dx=(ix-mapxind)*mapa.info.resolution;
						dy=(iy-mapyind)*mapa.info.resolution;
						dt=sqrt((dx*dx)+(dy*dy));
						ang=(atan2(dy,dx))-heading;
						index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
						localmap1.ranges[index]=dt;
						localmap1.intensities[index]=1;		//right lane
					} */
					if (mapa.data[MAP_IDX(mapa.info.width, ix, iy)]==45) {	//unknown lane
						dx=(ix-mapxind)*mapa.info.resolution;
						dy=(iy-mapyind)*mapa.info.resolution;
						dt=sqrt((dx*dx)+(dy*dy));
						ang=(atan2(dy,dx))-heading;
						index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
						if ((index!=0) && (index<1081)) {
							localmap1.ranges[index]=dt;
							localmap1.intensities[index]=0.5;
							}		//unknown lane
						if ((index>=180) && (index<300)) {
							x_right[index-180]=dx;
							y_right[index-180]=dy;
						}
						if ((index>780) && (index<=900)) {
							x_left[900-index]=dx;
							y_left[900-index]=dy;
						}
					}
					if (mapa.data[MAP_IDX(mapa.info.width, ix, iy)]==25) {	//red flag
						dx=(ix-mapxind)*mapa.info.resolution;
						dy=(iy-mapyind)*mapa.info.resolution;
						dt=sqrt((dx*dx)+(dy*dy));
						ang=(atan2(dy,dx))-heading;
						index=((ang-localmap1.angle_min)/localmap2.angle_increment);
						localmap2.ranges[index]=dt;
						localmap2.intensities[index]=0;		//red flag 
					}
					if (mapa.data[MAP_IDX(mapa.info.width, ix, iy)]==75) {	//blue flag
						dx=(ix-mapxind)*mapa.info.resolution;
						dy=(iy-mapyind)*mapa.info.resolution;
						dt=sqrt((dx*dx)+(dy*dy));
						ang=(atan2(dy,dx))-heading;
						index=((ang-localmap1.angle_min)/localmap2.angle_increment);
						localmap2.ranges[index]=dt;
						localmap2.intensities[index]=1;		//blue flag
					}
				
				}
				}
				
			} 
			for (unsigned int ya=1; ya<120; ya++) {
				if (y_right[120-ya]!=0 && y_right[119-ya]!=0 && x_right[120-ya]!=0 && x_right[119-ya]!=0) {
					angles_right[ya-1]= (atan2(y_right[120-ya]-y_right[119-ya],x_right[120-ya]-x_right[119-ya]));
				}					
				if (y_left[120-ya]!=0 && y_left[119-ya]!=0 && x_left[120-ya]!=0 && x_left[119-ya]!=0) {
					angles_left[ya-1]= (atan2(y_left[120-ya]-y_left[119-ya],x_left[120-ya]-x_left[119-ya]));
				}
			}
			for (unsigned int ya=0; ya<119; ya++) {
				n=0;
				m=0;
				if (angles_right[ya]!=0) {
					sumang_x=sumang_x+angles_right[ya];
					n=n+1;
				}
				if (angles_left[ya]!=0) {
					sumang_y=sumang_y+angles_left[ya];
					m=m+1;
				}
				
			}
			avgx=sumang_x/n;
			avgy=sumang_y/m;
			avgtot=((avgx+avgy)/2);
			goal_x=1.5*cos(avgtot)+robotx;		//1.5 meters
			goal_y=1.5*sin(avgtot)+roboty;
			mapxgind=floor(goal_x/mapa.info.resolution);
			mapygind=floor(goal_y/mapa.info.resolution);
			if (mapxgind>=0 && mapxgind<mapa.info.width && mapygind>=0 && mapygind<mapa.info.height) {
			mapa.data[MAP_IDX(mapa.info.width, mapxgind, mapygind)]=44;	//coment this line and change to publish
			}
			

ROS_INFO_STREAM("NO es el localmap");
	gpstail.x = robotx;
   	gpstail.y = roboty;
	gpstail.z = seq2;
	forgoal_gps.points.push_back(gpstail);
	seq2++;
	}

//Callback for lane lines
void laneCallback(const sensor_msgs::Image::ConstPtr& msg) {
//ROS_INFO_STREAM("Es el lane callback?");
//std_msgs/Header header
//uint32 height	
//uint32 width
//string encoding
//uint8 is_bigendian
//uint32 step		Full row length in bytes
//uint8[] data		actual matrix data, size is (step * rows)

		double lx, ly,gyll,gxll,a,b, glob_xleft, glob_yleft, glob_xright, glob_yright, beta, sumldx=0, sumldy=0, sumrdx=0, sumrdy=0, avgly=0, avglx=0, avgry=0, avgrx=0, angleft, angright, probaline;
		a=msg->step;
		b=msg->height;
		unsigned int mapxllind, mapyllind, mapxlind, mapylind, mapxrind, mapyrind; 
		for (unsigned int y =b-1; y > 0; y--) {
			for (unsigned int x = 0; x < a; x++) {
                lx=(b-y)*res;	//distance from robot x
                ly=res*(((a-1)/2)-x);	//distance from robot y
                gxll=(lx*cos(headingant[0]))-(ly*sin(headingant[0]))+robotxant[0]; //dist x from global
                gyll=(lx*sin(headingant[0]))+(ly*cos(headingant[0]))+robotyant[0]; //dist y from global
                mapxllind=floor(gxll/mapa.info.resolution);
                mapyllind=floor(gyll/mapa.info.resolution);
                if ((robotyant[0]!=0) && (robotxant[0]!=0) && mapxllind<mapa.info.width && mapyllind<mapa.info.height && (mapxllind>0) && (mapyllind>0)){
					if (msg->data[MAP_IDX(a, x, y)]==1 && (mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]<100)) {
                         mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]+=25;	//unknown lane line map
                    }
					else if (msg->data[MAP_IDX(a, x, y)]==0 && (mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]<30 && mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]>-120)  ) {
                        mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]-=1;
                    }
                    if (msg->data[MAP_IDX(a, x, y)]==1 && mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]>=55 && (mapa4.data[MAP_IDX(mapa4.info.width, mapxllind, mapyllind)]!=100) && (mapa5.data[MAP_IDX(mapa5.info.width, mapxllind, mapyllind)]!=100)) {
					    if (maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind, mapyllind)]<-5 || ( (map_prob_left[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]>map_prob_right[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]) && map_prob_left[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]>20)) {
                            mapa4.data[MAP_IDX(mapa4.info.width, mapxllind, mapyllind)]=100; //left
                            mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]=30;
                            for (unsigned int i=0; i<9; i++) {
                                dleftx[i]=dleftx[i+1];
                            }
                            dleftx[39]=gxll-gxlant;
                            dlefty[39]=gyll-gylant;
                            gxlant=gxll;
                            gylant=gyll;
                            //compute left region
                            for (unsigned int i =0; i<9 ; i++) {
                                for (unsigned int j =0; j < 9; j++) {
                                    if ((mapxllind+i-4)<mapa.info.width && (mapyllind+j-4)<mapa.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                        maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=-10;
                                    }
                                }
                            }
                        }
					    if (maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind, mapyllind)]>5) {
                            mapa5.data[MAP_IDX(mapa5.info.width, mapxllind, mapyllind)]=100; //right
                            mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]=60;
                            for (unsigned int i=0; i<9; i++) {
                                drightx[i]=drightx[i+1];
                            }
                            drightx[39]=gxll-gxrant;
                            drighty[39]=gyll-gyrant;
                            gxrant=gxll;
                            gyrant=gyll;
                            //compute right region
                            for (unsigned int i =0; i<9 ; i++) {
                                for (unsigned int j =0; j < 9; j++) {
                                    if ((mapxllind+i-4)<mapa.info.width && (mapyllind+j-4)<mapa.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                    maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
                                    //mapa.data[MAP_IDX(mapa4.info.width, mapxllind+i-4, mapyllind+j-4)]=45; //test
                                    }
                                }
                            }
					    }
					
					
					//****Strategy to classify first pixels as L or R lines****
						if (firstL==0) {
							beta=atan2(ly,lx);
							if (beta>=0.78) {
							mapa4.data[MAP_IDX(mapa4.info.width, mapxllind, mapyllind)]=100;
							firstL=1;
							gxlant=gxll;
							gylant=gyll;
							//compute left region
							for (unsigned int i =0; i<9 ; i++) {
								for (unsigned int j =0; j < 9; j++) {
									if ((mapxllind+i-4)<mapa.info.width && (mapyllind+j-4)<mapa.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
									maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=-10;
									}
								}
							}
							}
						}
						if (firstR==0) {
							beta=atan2(ly,lx);
							if (beta<=-0.78) {
							mapa5.data[MAP_IDX(mapa5.info.width, mapxllind, mapyllind)]=100;
							firstR=1;
							gxrant=gxll;
							gyrant=gyll;
							//compute right region
							for (unsigned int i =0; i<9 ; i++) {
								for (unsigned int j =0; j < 9; j++) {
									if ((mapxllind+i-4)<mapa.info.width && (mapyllind+j-4)<mapa.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
									maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
									}
								}
							}
							}
						}

			/*	if (mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]==10) {
				mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]=30;	//general map: left lane
				mapa4.data[MAP_IDX(mapa4.info.width, mapxllind, mapyllind)]=60;	//left lane map
				ROS_INFO_STREAM("left ");
				//save in queue
					leftlane.x = gxll;
   					leftlane.y = gyll;
			 		leftlane.z = seq;
					forgoal_left.points.push_back(leftlane);
				}
				else if (mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]==20) {
				mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]=60;	//general map: right lane
				mapa5.data[MAP_IDX(mapa5.info.width, mapxllind, mapyllind)]=60;	//right lane map
				ROS_INFO_STREAM("right ");
				//save in queue
					rightlane.x = gxll;
   					rightlane.y = gyll;
			 		rightlane.z = seq;
					forgoal_right.points.push_back(leftlane);
				}
			*/ //	else {
				
				//mapa.data[MAP_IDX(mapa.info.width, mapxllind, mapyllind)]=45;	//general map: unknown lane
				mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]=120;	//mapa 3 unknown lane
				}

					//}

					
					}  //end of boundaries check
				//} //end of general "if"
			} //end of first for
			seq++;		
		} //end of second for
		for (unsigned int i = 0; i< (mapa.info.width*mapa.info.height); i++) {
			if (mapa.data[i]==22)
			mapa.data[i]=0;
		}
		for (unsigned int i=0; i<40 ; i++) {
			sumldx=sumldx+ dleftx[i];
			sumldy=sumldy+ dlefty[i];
		}
		for (unsigned int i=0; i<40 ; i++) {
			sumrdx=sumrdx+ drightx[i];
			sumrdy=sumrdy+ drighty[i];
		}
		avglx=sumldx/40;
		avgly=sumldy/40;
		avgrx=sumrdx/40;
		avgry=sumrdy/40;
		angleft=atan2(avgly,avglx);
		angright=atan2(avgry,avgrx);
	if ((((angleft-angleftant)<2.35 && (angleft-angleftant)>-2.35) || ((angleft-angleftant)>3.9 || (angleft-angleftant)<-3.9)) || firstangleleft==0) //si el angulo esta chido procede
	{
		for (double predleft=mapa.info.resolution; predleft <15 ; predleft=predleft+mapa.info.resolution) {
			for (int anguleitor=-32; anguleitor<32; anguleitor++)
				{
					glob_xleft=predleft*cos(angleft+anguleitor*(3.1416/360)) + gxlant;
					glob_yleft=predleft*sin(angleft+anguleitor*(3.1416/360)) + gylant;
					mapxlind=floor(glob_xleft/mapa.info.resolution);
					mapylind=floor(glob_yleft/mapa.info.resolution);

					if ( glob_yleft>0 && glob_xleft>0 && glob_xleft<mapa.info.width*mapa.info.resolution && glob_yleft<mapa.info.height*mapa.info.resolution ) {
						
							//TODO poner la funcion que hace ambas gausianas e incrementar
							probaline=(exp(-(pow((predleft/8),2))))*(exp(-(pow((anguleitor/10),2))));
							map_prob_left[MAP_IDX(mapa.info.width, mapxlind, mapylind)]+=probaline;
							mapa7.data[MAP_IDX(mapa7.info.width, mapxlind, mapylind)]+=1;
							
							//mapa.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]=22; //test
					}
			}
		}

ROS_INFO_STREAM("left: " << angleft);
ROS_INFO_STREAM("leftant: " << angleftant);
		angleftant=angleft;
		firstangleleft=1;
	}
	else {
		for (unsigned int i=35; i<40; i++) {
			dleftx[i]=dleftx[34];
			dlefty[i]=dlefty[34];
		}
	}


	if ((((angright-angrightant)<2.35 && (angright-angrightant)>-2.35) || ((angright-angrightant)>3.9 || (angright-angrightant)<-3.9)) || firstangleright==0) //si el angulo esta chido procede
	{
		for (double predright=mapa.info.resolution; predright <10 ; predright=predright+mapa.info.resolution) {

					glob_xright=predright*cos(angright+0) + gxrant;
					glob_yright=predright*sin(angright+0) + gyrant;
					mapxlind=floor(glob_xright/mapa.info.resolution);
					mapylind=floor(glob_yright/mapa.info.resolution);

					if ( glob_yright>0 && glob_xright>0 && glob_xright<mapa.info.width*mapa.info.resolution && glob_yright<mapa.info.height*mapa.info.resolution ) {
						
						
				
							mapa8.data[MAP_IDX(mapa8.info.width, mapxlind, mapylind)]=100;
							//mapa.data[MAP_IDX(mapa.info.width, mapxlind, mapylind)]=22; //test
					}
		}
ROS_INFO_STREAM("right: " << angright);
ROS_INFO_STREAM("rightant: " << angrightant);
		angrightant=angright;
		firstangleright=1;
	}
	else {
		for (unsigned int i=35; i<40; i++) {
			drightx[i]=drightx[34];
			drighty[i]=drighty[34];
		}
	}

//ROS_INFO_STREAM(dleftx[39]);


} // end of callback


int main(int argc, char **argv) {
//ROS_INFO_STREAM("X");
	ros::init(argc, argv, "the_mapping");
	ros::NodeHandle n;
	

   // geometry_msgs::Point32 leftlane;
   // leftlane.x = 10;
   // leftlane.y = 9;
   // leftlane.z = 8;
   // forgoal.points.push_back(leftlane);
   // leftlane.x = 7;
   // leftlane.y = 6;
   // leftlane.z = 5;
   // forgoal.points.push_back(leftlane);

//MAPS PARAMETERS
	//MAP 1 (GENERAL MAP)
	mapa.info.resolution=0.1;
	mapa.info.width=800;
	mapa.info.height=800;
	mapa.info.origin.position.x=0.0;
	mapa.info.origin.position.y=0.0;
	mapa.info.origin.position.z=0.0;
	mapa.info.origin.orientation.x=0.0;
	mapa.info.origin.orientation.y=0.0;
	mapa.info.origin.orientation.z=0.0;
	mapa.info.origin.orientation.w=1.0;
	mapa.data.resize(mapa.info.width * mapa.info.height);
	//MAP 2 (OBSTACLE Thresholded map)
	mapa2.info.resolution=0.1;
	mapa2.info.width=800;
	mapa2.info.height=800;
	mapa2.info.origin.position.x=0.0;
	mapa2.info.origin.position.y=0.0;
	mapa2.info.origin.position.z=0.0;
	mapa2.info.origin.orientation.x=0.0;
	mapa2.info.origin.orientation.y=0.0;
	mapa2.info.origin.orientation.z=0.0;
	mapa2.info.origin.orientation.w=1.0;
	mapa2.data.resize(mapa2.info.width * mapa2.info.height);
	//MAP 3 (UNKNOWN Lane lines map)
	mapa3.info.resolution=0.1;
	mapa3.info.width=800;
	mapa3.info.height=800;
	mapa3.info.origin.position.x=0.0;
	mapa3.info.origin.position.y=0.0;
	mapa3.info.origin.position.z=0.0;
	mapa3.info.origin.orientation.x=0.0;
	mapa3.info.origin.orientation.y=0.0;
	mapa3.info.origin.orientation.z=0.0;
	mapa3.info.origin.orientation.w=1.0;
	mapa3.data.resize(mapa3.info.width * mapa3.info.height);
	//MAP regions  (to classify lane lines)
	maparegiones.info.resolution=0.1;
	maparegiones.info.width=800;
	maparegiones.info.height=800;
	maparegiones.info.origin.position.x=0.0;
	maparegiones.info.origin.position.y=0.0;
	maparegiones.info.origin.position.z=0.0;
	maparegiones.info.origin.orientation.x=0.0;
	maparegiones.info.origin.orientation.y=0.0;
	maparegiones.info.origin.orientation.z=0.0;
	maparegiones.info.origin.orientation.w=1.0;
	maparegiones.data.resize(maparegiones.info.width * maparegiones.info.height);

	//MAP 4 (Left Lane lines map)
	mapa4.info.resolution=0.1;
	mapa4.info.width=800;
	mapa4.info.height=800;
	mapa4.info.origin.position.x=0.0;
	mapa4.info.origin.position.y=0.0;
	mapa4.info.origin.position.z=0.0;
	mapa4.info.origin.orientation.x=0.0;
	mapa4.info.origin.orientation.y=0.0;
	mapa4.info.origin.orientation.z=0.0;
	mapa4.info.origin.orientation.w=1.0;
	mapa4.data.resize(mapa4.info.width * mapa4.info.height);
	//MAP 5 (Right Lane lines map)
	mapa5.info.resolution=0.1;
	mapa5.info.width=800;
	mapa5.info.height=800;
	mapa5.info.origin.position.x=0.0;
	mapa5.info.origin.position.y=0.0;
	mapa5.info.origin.position.z=0.0;
	mapa5.info.origin.orientation.x=0.0;
	mapa5.info.origin.orientation.y=0.0;
	mapa5.info.origin.orientation.z=0.0;
	mapa5.info.origin.orientation.w=1.0;
	mapa5.data.resize(mapa5.info.width * mapa5.info.height);
	//MAP 6 (POSITIONS)
	mapa6.info.resolution=0.1;
	mapa6.info.width=800;
	mapa6.info.height=800;
	mapa6.info.origin.position.x=0.0;
	mapa6.info.origin.position.y=0.0;
	mapa6.info.origin.position.z=0.0;
	mapa6.info.origin.orientation.x=0.0;
	mapa6.info.origin.orientation.y=0.0;
	mapa6.info.origin.orientation.z=0.0;
	mapa6.info.origin.orientation.w=1.0;
	mapa6.data.resize(mapa6.info.width * mapa6.info.height);
	//MAP 7 (Predicted left)
	mapa7.info.resolution=0.1;
	mapa7.info.width=800;
	mapa7.info.height=800;
	mapa7.info.origin.position.x=0.0;
	mapa7.info.origin.position.y=0.0;
	mapa7.info.origin.position.z=0.0;
	mapa7.info.origin.orientation.x=0.0;
	mapa7.info.origin.orientation.y=0.0;
	mapa7.info.origin.orientation.z=0.0;
	mapa7.info.origin.orientation.w=1.0;
	mapa7.data.resize(mapa7.info.width * mapa7.info.height);
	//MAP 8 (Predicted right)
	mapa8.info.resolution=0.1;
	mapa8.info.width=800;
	mapa8.info.height=800;
	mapa8.info.origin.position.x=0.0;
	mapa8.info.origin.position.y=0.0;
	mapa8.info.origin.position.z=0.0;
	mapa8.info.origin.orientation.x=0.0;
	mapa8.info.origin.orientation.y=0.0;
	mapa8.info.origin.orientation.z=0.0;
	mapa8.info.origin.orientation.w=1.0;
	mapa8.data.resize(mapa8.info.width * mapa8.info.height);

	//LOCALMAP lanelines
	localmap1.angle_min=-2.35837626457;
	localmap1.angle_max=2.35837626457;
	localmap1.angle_increment=0.00436736317351;
	localmap1.time_increment=0;
	localmap1.scan_time=0;
	localmap1.range_min=0;
	localmap1.range_max=20;
	localmap1.ranges.resize(1081);
	localmap1.intensities.resize(1081);

	//LOCALMAP flags
	localmap2.angle_min=-2.35837626457;
	localmap2.angle_max=2.35837626457;
	localmap2.angle_increment=0.00436736317351;
	localmap2.time_increment=0;
	localmap2.scan_time=0;
	localmap2.range_min=0;
	localmap2.range_max=20;
	localmap2.ranges.resize(1081);
	localmap2.intensities.resize(1081);

	
	
	//******INITIALIZE MAPS
	for (unsigned int i = 0; i< (mapa.info.width*mapa.info.height); i++) {
		mapa.data[i]=50;
		mapa2.data[i]=50;
		mapa3.data[i]=0;
		mapa4.data[i]=0;
		mapa5.data[i]=0;
		mapa6.data[i]=0;
		mapa7.data[i]=0;
		mapa8.data[i]=0;
		maparegiones.data[i]=0;
		map_grid[i]=0;
		map_prob_left[i]=0;
		map_prob_right[i]=0;
	}
	//******INITIALIZE lanelines deltas
	for (unsigned int i = 0; i< 10; i++) {
		dleftx[i]=0;
		dlefty[i]=0;
		drightx[i]=0;
		drighty[i]=0;
	}
	//*******Initialize last 15 poses*******
	for (unsigned int i= 0; i< 2; i++) {
		robotxant[i]=0;
		robotyant[i]=0;
		headingant[i]=0;
	}
	
ROS_INFO_STREAM("corre");
	for (unsigned int lmi=0; lmi < 1081; lmi++) {
	
		localmap1.ranges[lmi]=0;
		localmap2.ranges[lmi]=0;
		localmap1.intensities[lmi]=0.5;
		localmap2.intensities[lmi]=0.5;
	}
	//*******Maps initialized*****




	//SUBSCRIBE/PUBLISH TO:
	ros::Subscriber laserSub = n.subscribe("base_scan", 1, laserCallback);
	//ros::Subscriber laserSub = n.subscribe("lidar/scan", 1, laserCallback); //For Husky
	//ros::Subscriber initPoseSub = n.subscribe("initial_pose", 1, initCallback);//initial pose (husky)
	ros::Subscriber poseSub = n.subscribe("base_pose_ground_truth", 1, 	poseCallback);	//For stage
	//ros::Subscriber poseSub = n.subscribe("odom_combined", 1, poseCallback);	//For husky
    ros::Subscriber laneSub = n.subscribe("image_out", 1, laneCallback);	//change to Tristans message

	ros::Publisher mappingPub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);	//GEneral map 

	ros::Publisher localmap1Pub = n.advertise<sensor_msgs::LaserScan>("localmap1", 1); //Local1 map (lanelines)
	ros::Publisher localmap2Pub = n.advertise<sensor_msgs::LaserScan>("localmap2", 1); //Local2 map (flags)
	ros::Publisher forgoalPub = n.advertise<geometry_msgs::Polygon>("forgoal", 1); //vectors/arrays/structures for goal selection
    //ros::Publisher polygon_pub;


	ros::Rate rate(45);
	
	while(ros::ok()) {
		
		 //img=cvLoadImage("frame0000.jpg");

		mappingPub.publish(mapa);
		//localmap1Pub.publish(localmap1);
		//localmap2Pub.publish(localmap2);
		//forgoalPub.publish(forgoal_left);
		//forgoalPub.publish(forgoal_right);
		//forgoalPub.publish(forgoal_gps);
		ROS_INFO("Most perfect map beta cl  predt 1.2");

	//ofstream myfile;
	//	myfile.open ("mapa.txt");
	//	myfile << mapa ;
	//	myfile.close();

	//ofstream myfile2;
	//	myfile2.open ("mapa3.txt");
	//	myfile2 << mapa3;
	//	myfile2.close();
	


		
		ros::spinOnce();
		rate.sleep();
	
	}

}
