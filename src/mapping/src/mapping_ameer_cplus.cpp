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
#include "nav_msgs/GetMap.h" //This is used in the service call for gmapping to get map
#include <math.h>                                       //Library to perform math operations
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include </usr/include/bullet/LinearMath/btQuaternion.h>                    // Library needed to convert  quaterions
#include </usr/include/bullet/LinearMath/btMatrix3x3.h>                     //  into Euler angles
//#include </opt/ros/hydro/include/tf/LinearMath/Matrix3x3.h>
#include <stdio.h>                                      //not used
#include <opencv2/opencv.hpp>                           //not used
#include "opencv2/core/core.hpp"                        //not used
#include <opencv2/highgui/highgui.hpp>                  //not used
#include <opencv/cv.h>                                  //not used
//A#include <the_mapping/ForGoal.h>                        // Library created to send pose messages of goals
#include "opencv/ml.h"                                  //not used
#include "opencv/cxcore.h"                              //not used
#include "opencv/highgui.h"                             //not used

#include <tf/transform_listener.h>
//********************************************************************************************************************************************

using namespace std;                        //Don't remember exactly why I included these
using namespace cv;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
//This function makes indexing easier.
// The output will be the index number of a vector corresponding to
// the element (i,j) in a matrix. sx=number of columns of the map/image.
// i=element i of the matrix
// j=element j of the matrix


// Full map table - Retrieved From Paper
// 0 = Free Cell
// 10 = Predicted Left Lane
// 20 = Predicted Right Lane
// 30 = Left Lane
// 45 = Unknown Lane
// 50 = Uknown ?
// 60 = Right Lane
// 85 = Position
// 100 = Obstacle

//*****************************************VARIABLES USED***********************************************************************************
double pose_xinit, pose_yinit, pose_ainit;              //intitial position of the robot values in UTM //(used in alternate pose callback)
int iy2, ix2;                                           //
//float robotx=0, roboty=0, heading=0, pitch=0, roll=0;  //robotx and roboty are the current position of the robot in meters in global coordinates
double map_prob_left[1000000];                          //a map storing all the probabilities of a lane line to be part of the left line
double map_prob_right[1000000];                         //a map storing all the probabilities of a lane line to be part of the right linelanes
double robotxant [2];                                   //a vector storing the last 2 positions of the robot (x coordinates)
double robotyant [2];                                   //a vector storing the last 2 positions of the robot (y coordinates)
double headingant [2];                                  //a vector storing the last 2 positions of the robot (heding in degrees)
double dleftx[40];                                      //last 40 cells classified as left (x coordinates)
double dlefty[40];                                      //last 40 cells classified as left (y coordinates)
double drightx[40];                                     //last 40 cells classified as right (x coordinates
double drighty[40];                                     //last 40 cells classified as right (y coordinates
double gylant, gxlant, gxrant, gyrant, angleftant=0, angrightant=0, firstangleleft=0, firstangleright=0;
float res=0.020;		                                //Resolution of lane line images meters/px
unsigned int seq=0;		                //sequence for lanelines goal (not used)
unsigned int seq2=0;		            //sequence for gps tail (not used)
unsigned int firstL=0, firstR=0;                        // flags to know if first regions Left and right are already classified/identified
nav_msgs::OccupancyGrid full_map;                           // General Map ...this is "THA MAP"! (includes all the features: obstacles, lane lines, positions...)
nav_msgs::OccupancyGrid obsta_map;                          //Obstacles map ...TRESHOLDED
nav_msgs::OccupancyGrid mapa3;                          //Lane lines map linelanes
nav_msgs::OccupancyGrid l_lane_map;                          //Map of left lanes
nav_msgs::OccupancyGrid r_lane_map;                          //Map of right lanes
nav_msgs::OccupancyGrid posit_map;                          //Positions map
nav_msgs::OccupancyGrid mapa7;                          //Predicted left
nav_msgs::OccupancyGrid mapa8;                          //Predicted right
nav_msgs::OccupancyGrid maparegiones;               //Map of regions to classify left or right lanes (negative for left, positive for right)
nav_msgs::OccupancyGrid localmap_occup1;            //local map occupancy grid 1
sensor_msgs::LaserScan localmap1;                   //Local map in LIDAR format
sensor_msgs::LaserScan localmap2;                   //Local map in LIDAR format
geometry_msgs::Polygon forgoal_left;                //not used
geometry_msgs::Polygon forgoal_right;               //not used
geometry_msgs::Polygon forgoal_gps;                 //not used
geometry_msgs::Point32 leftlane;                    //not used
geometry_msgs::Point32 rightlane;                   //not used
geometry_msgs::Point32 gpstail;                     //not used
bool setup=true;
//***************************************End of global variables**************************************************************************************



void laneCallback(const sensor_msgs::Image::ConstPtr& msg) {
    //std_msgs/Header header
    //uint32 height
    //uint32 width
    //string encoding
    //uint8 is_bigendian
    //uint32 step		Full row length in bytes
    //uint8[] data		actual matrix data, size is (step * rows)

    double lx, ly,gyll,gxll,a,b, glob_xleft, glob_yleft;
    double glob_xright, glob_yright, beta, sumldx=0, sumldy=0, sumrdx=0;
    double sumrdy=0, avgly=0, avglx=0, avgry=0, avgrx=0, angleft, angright, probaline;
    a=msg->step;
    b=msg->height;
    unsigned int mapxllind, mapyllind, mapxlind, mapylind, mapxrind, mapyrind;
    for (unsigned int y =b-1; y > 0; y--) {
        for (unsigned int x = 0; x < a; x++) {
            lx=(b-y)*res;	//distance from robot x
            ly=res*(((a-1)/2)-x);	//distance from robot y
            gxll=(lx*cos(headingant[0]))-(ly*sin(headingant[0]))+robotxant[0]; //dist x from global
            gyll=(lx*sin(headingant[0]))+(ly*cos(headingant[0]))+robotyant[0]; //dist y from global
            mapxllind=floor(gxll/full_map.info.resolution);
            mapyllind=floor(gyll/full_map.info.resolution);
            if ((robotyant[0]!=0) && (robotxant[0]!=0) && mapxllind<full_map.info.width && mapyllind<full_map.info.height && (mapxllind>0) && (mapyllind>0)){
                if (msg->data[MAP_IDX(a, x, y)]==1 && (mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]<100)) {
                    mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]+=25;	//unknown lane line map
                }
                else if (msg->data[MAP_IDX(a, x, y)]==0 && (mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]<30 && mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]>-120)  ) {
                    mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]-=1;
                }
                if (msg->data[MAP_IDX(a, x, y)]==1 && mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]>=55 && (l_lane_map.data[MAP_IDX(l_lane_map.info.width, mapxllind, mapyllind)]!=100) && (r_lane_map.data[MAP_IDX(r_lane_map.info.width, mapxllind, mapyllind)]!=100)) {
                    if (maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind, mapyllind)]<-5 || ( (map_prob_left[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]>map_prob_right[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]) && map_prob_left[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]>20)) {
                        l_lane_map.data[MAP_IDX(l_lane_map.info.width, mapxllind, mapyllind)]=100; //left
                        full_map.data[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]=30;
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
                                if ((mapxllind+i-4)<full_map.info.width && (mapyllind+j-4)<full_map.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                    maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=-10;
                                }
                            }
                        }
                    }
                    if (maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind, mapyllind)]>5) {
                        r_lane_map.data[MAP_IDX(r_lane_map.info.width, mapxllind, mapyllind)]=100; //right
                        full_map.data[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]=60;
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
                                if ((mapxllind+i-4)<full_map.info.width && (mapyllind+j-4)<full_map.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                    maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
                                    //full_map.data[MAP_IDX(mapa4.info.width, mapxllind+i-4, mapyllind+j-4)]=45; //test
                                }
                            }
                        }
                    }


                    //****Strategy to classify first pixels as L or R lines****
                    if (firstL==0) {
                        beta=atan2(ly,lx);
                        if (beta>=0.78) {
                            l_lane_map.data[MAP_IDX(l_lane_map.info.width, mapxllind, mapyllind)]=100;
                            firstL=1;
                            gxlant=gxll;
                            gylant=gyll;
                            //compute left region
                            for (unsigned int i =0; i<9 ; i++) {
                                for (unsigned int j =0; j < 9; j++) {
                                    if ((mapxllind+i-4)<full_map.info.width && (mapyllind+j-4)<full_map.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                        maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=-10;
                                    }
                                }
                            }
                        }
                    }
                    if (firstR==0) {
                        beta=atan2(ly,lx);
                        if (beta<=-0.78) {
                            r_lane_map.data[MAP_IDX(r_lane_map.info.width, mapxllind, mapyllind)]=100;
                            firstR=1;
                            gxrant=gxll;
                            gyrant=gyll;
                            //compute right region
                            for (unsigned int i =0; i<9 ; i++) {
                                for (unsigned int j =0; j < 9; j++) {
                                    if ((mapxllind+i-4)<full_map.info.width && (mapyllind+j-4)<full_map.info.height && (mapxllind+i-4)>0 && (mapyllind+j-4)>0) {
                                        maparegiones.data[MAP_IDX(maparegiones.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
                                    }
                                }
                            }
                        }
                    }
                    mapa3.data[MAP_IDX(mapa3.info.width, mapxllind, mapyllind)]=120;	//mapa 3 unknown lane
                }
            }  //end of boundaries check
        } //end of first for
        seq++;
    } //end of second for
    for (unsigned int i = 0; i< (full_map.info.width*full_map.info.height); i++) {
        if (full_map.data[i]==22)
            full_map.data[i]=0;
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
        for (double predleft=full_map.info.resolution; predleft <15 ; predleft=predleft+full_map.info.resolution) {
            for (int anguleitor=-32; anguleitor<32; anguleitor++)
            {
                glob_xleft=predleft*cos(angleft+anguleitor*(3.1416/360)) + gxlant;
                glob_yleft=predleft*sin(angleft+anguleitor*(3.1416/360)) + gylant;
                mapxlind=floor(glob_xleft/full_map.info.resolution);
                mapylind=floor(glob_yleft/full_map.info.resolution);

                if ( glob_yleft>0 && glob_xleft>0 && glob_xleft<full_map.info.width*full_map.info.resolution && glob_yleft<full_map.info.height*full_map.info.resolution ) {

                    //TODO poner la funcion que hace ambas gausianas e incrementar
                    probaline=(exp(-(pow((predleft/8),2))))*(exp(-(pow((anguleitor/10),2))));
                    map_prob_left[MAP_IDX(full_map.info.width, mapxlind, mapylind)]+=probaline;
                    mapa7.data[MAP_IDX(mapa7.info.width, mapxlind, mapylind)]+=1;

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
        for (double predright=full_map.info.resolution; predright <10 ; predright=predright+full_map.info.resolution) {

            glob_xright=predright*cos(angright+0) + gxrant;
            glob_yright=predright*sin(angright+0) + gyrant;
            mapxlind=floor(glob_xright/full_map.info.resolution);
            mapylind=floor(glob_yright/full_map.info.resolution);

            if ( glob_yright>0 && glob_xright>0 && glob_xright<full_map.info.width*full_map.info.resolution && glob_yright<full_map.info.height*full_map.info.resolution ) {
                mapa8.data[MAP_IDX(mapa8.info.width, mapxlind, mapylind)]=100;
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
} // end of callback



void retrieveMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO_STREAM("GOT MAP");
    //the logical way to set this up would be to get the map from gmapping first
    // and then initilize everything based off of that
    full_map.info = msg->info;
    full_map.header = msg->header;
    full_map.data = msg->data;
    // In gmapping 0 is a free cell, -1 an unknown spot, 100 is an obstacle
    if(setup){ //if this i our first time gettig the maplets init everything based off of it
        localmap_occup1.info.resolution = msg->info.resolution;
        localmap_occup1.info.width = 50; //Change it 10 meters on each side
        localmap_occup1.info.height = 50;
        localmap_occup1.data.resize(localmap_occup1.info.width * localmap_occup1.info.height);
        for (unsigned int i = 0; i< (localmap_occup1.info.width*localmap_occup1.info.height); i++) {
            localmap_occup1.data[i]=0;
        }
        setup = false;
    }

}



void extractLocalMap(const nav_msgs::Odometry::ConstPtr& msg){
    /*
    To know where the robot is in the map, you want to transform the origin of the base_link frame into the map frame.
    In addition to the map, SLAM systems (gmapping and karto) publish transform data. In particular, they provide the
    transform from the map frame to the odom frame. Another node (which one will depend on what kind of robot you're using)
     should already be publishing the transform from the odom frame to the base_link frame.

    rosrun tf tf_echo map base_link
    Given the robot's pose in the map frame, if you want the corresponding index into the occupancy grid map, you'd do something like below
     */
    //testing stuff out
    float dx=0;
    float dy=0;
    float dt=0;
    float heading=0;
    float pitch;
    float roll;
    float ang;
    unsigned int index;
    btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
    			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
    			msg->pose.pose.orientation.w);
    btMatrix3x3(q).getEulerYPR(heading, pitch, roll);

    //end testing


    for (unsigned int lmi=0; lmi < 1081; lmi++) {

        localmap1.ranges[lmi]=0;
        localmap2.ranges[lmi]=0;
        localmap1.intensities[lmi]=0.5;
        localmap2.intensities[lmi]=0.5;
    }



    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    double map_x = transform.getOrigin().x();
    double map_y = transform.getOrigin().y();
    double map_z = transform.getOrigin().z();
    ROS_INFO_STREAM("X" << transform.getOrigin().x());
    ROS_INFO_STREAM("Y" << transform.getOrigin().y());

    int grid_x;
    int grid_y;

    grid_x = (unsigned int)((map_x - full_map.info.origin.position.x) / full_map.info.resolution);
    grid_y = (unsigned int)((map_y - full_map.info.origin.position.y) / full_map.info.resolution);

    ROS_INFO_STREAM("gridx " << grid_x);
    ROS_INFO_STREAM("gridy " << grid_y);
    localmap_occup1.info.origin.position.x=map_x-25*full_map.info.resolution;
    localmap_occup1.info.origin.position.y=map_y-25*full_map.info.resolution;
    localmap_occup1.info.origin.position.z=map_z;


    ix2=(grid_x-25);	//in map coordinates
    iy2=(grid_y-25); 	// in map coordinates
    //UPDATE LOCAL MAPS
    // you were disconnected
    ROS_INFO_STREAM("ix2 " << ix2);
    ROS_INFO_STREAM("iy2 " << iy2);

    int i = 0;
    for (int iy=iy2; iy<(iy2+50); iy++) {
        for (int ix=ix2; ix<(ix2+50); ix++) {
            //ROS_INFO_STREAM("shit: " <<ix+(localmap_occup1.info.origin.position.x/full_map.info.resolution));
            localmap_occup1.data[i] = full_map.data[MAP_IDX(full_map.info.width, ix, iy)];
            if(full_map.data[MAP_IDX(full_map.info.width, ix, iy)]==100) {	//left lane
                dx=(ix-grid_x)*full_map.info.resolution;
                dy=(iy-grid_y)*full_map.info.resolution;
                dt=sqrt((dx*dx)+(dy*dy));
                ang=(atan2(dy,dx));
                index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
                if(index<5000000){
                    localmap1.ranges[index]=dt;
                    localmap1.intensities[index]=0;		//left lane
                }
            }
            ros::Time scan_time = ros::Time::now();
            localmap1.header.stamp = scan_time;
            localmap1.header.frame_id = "base_laser_link";

            i++;
        }
    }

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "the_mapping");
    ros::NodeHandle n;
    //MAPS PARAMETERs



    //MAP 2 (OBSTACLE Thresholded map)
    obsta_map.info.resolution=0.1;
    obsta_map.info.width=800;
    obsta_map.info.height=800;
    obsta_map.info.origin.position.x=0.0;
    obsta_map.info.origin.position.y=0.0;
    obsta_map.info.origin.position.z=0.0;
    obsta_map.info.origin.orientation.x=0.0;
    obsta_map.info.origin.orientation.y=0.0;
    obsta_map.info.origin.orientation.z=0.0;
    obsta_map.info.origin.orientation.w=1.0;
    obsta_map.data.resize(obsta_map.info.width * obsta_map.info.height);
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
    l_lane_map.info.resolution=0.1;
    l_lane_map.info.width=800;
    l_lane_map.info.height=800;
    l_lane_map.info.origin.position.x=0.0;
    l_lane_map.info.origin.position.y=0.0;
    l_lane_map.info.origin.position.z=0.0;
    l_lane_map.info.origin.orientation.x=0.0;
    l_lane_map.info.origin.orientation.y=0.0;
    l_lane_map.info.origin.orientation.z=0.0;
    l_lane_map.info.origin.orientation.w=1.0;
    l_lane_map.data.resize(l_lane_map.info.width * l_lane_map.info.height);
    //MAP 5 (Right Lane lines map)
    r_lane_map.info.resolution=0.1;
    r_lane_map.info.width=800;
    r_lane_map.info.height=800;
    r_lane_map.info.origin.position.x=0.0;
    r_lane_map.info.origin.position.y=0.0;
    r_lane_map.info.origin.position.z=0.0;
    r_lane_map.info.origin.orientation.x=0.0;
    r_lane_map.info.origin.orientation.y=0.0;
    r_lane_map.info.origin.orientation.z=0.0;
    r_lane_map.info.origin.orientation.w=1.0;
    r_lane_map.data.resize(r_lane_map.info.width * r_lane_map.info.height);
    //MAP 6 (POSITIONS)
    posit_map.info.resolution=0.1;
    posit_map.info.width=800;
    posit_map.info.height=800;
    posit_map.info.origin.position.x=0.0;
    posit_map.info.origin.position.y=0.0;
    posit_map.info.origin.position.z=0.0;
    posit_map.info.origin.orientation.x=0.0;
    posit_map.info.origin.orientation.y=0.0;
    posit_map.info.origin.orientation.z=0.0;
    posit_map.info.origin.orientation.w=1.0;
    posit_map.data.resize(posit_map.info.width * posit_map.info.height);
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
    for (unsigned int i = 0; i< (full_map.info.width*full_map.info.height); i++) {
        obsta_map.data[i]=50;
        mapa3.data[i]=0;
        l_lane_map.data[i]=0;
        r_lane_map.data[i]=0;
        posit_map.data[i]=0;
        mapa7.data[i]=0;
        mapa8.data[i]=0;
        maparegiones.data[i]=0;
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
    for (unsigned int lmi=0; lmi < 1081; lmi++) {

        localmap1.ranges[lmi]=0;
        localmap2.ranges[lmi]=0;
        localmap1.intensities[lmi]=0.5;
        localmap2.intensities[lmi]=0.5;
    }


    //*******Maps initialized*****


    //SUBSCRIBE/PUBLISH TO:
    //ros::Subscriber initPoseSub = n.subscribe("initial_pose", 1, initCallback);//initial pose (husky)
    ros::Subscriber poseSub = n.subscribe("base_pose_ground_truth", 1, 	extractLocalMap);	//change this so it subscribes to the tf
    //Aros::Subscriber laneSub = n.subscribe("image_out", 1, laneCallback);	//change to Tristans message
    ros::Subscriber mapSub = n.subscribe("map", 1, retrieveMapCallback); //Subscribe to gmapping map
    ros::Publisher mappingPub = n.advertise<nav_msgs::OccupancyGrid>("full_map", 1);	//Publish the Full map
    ros::Publisher localmappingPub = n.advertise<nav_msgs::OccupancyGrid>("localmap_occup1", 1);	//Publish the local map

    ros::Publisher localmap1Pub = n.advertise<sensor_msgs::LaserScan>("localmap1", 1); //Local1 map (lanelines)
    //ros::Publisher localmap2Pub = n.advertise<sensor_msgs::LaserScan>("localmap2", 1); //Local2 map (flags)
    //Aros::Publisher forgoalPub = n.advertise<geometry_msgs::Polygon>("forgoal", 1); //vectors/arrays/structures for goal selection

    ros::Rate rate(45);

    while(ros::ok()) {
        mappingPub.publish(full_map);
        localmappingPub.publish(localmap_occup1);
        localmap1Pub.publish(localmap1);
        //localmap1Pub.publish(localmap1);
        //localmap2Pub.publish(localmap2);
        //forgoalPub.publish(forgoal_left);
        //forgoalPub.publish(forgoal_right);
        //forgoalPub.publish(forgoal_gps);
        //ROS_INFO_STREAM("ros is ok");

        ros::spinOnce();
        rate.sleep();
    }
}
