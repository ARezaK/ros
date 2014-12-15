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
#include <unistd.h>
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
float robotx=0, roboty=0, heading=0, pitch=0, roll=0;  //robotx and roboty are the current position of the robot in meters in global coordinates
double map_prob_left[19219456];                          //a map storing all the probabilities of a lane line to be part of the left line
double map_prob_right[19219456];                         //a map storing all the probabilities of a lane line to be part of the right linelanes
double robotxant [2];                                   //a vector storing the last 2 positions of the robot (x coordinates)
double robotyant [2];                                   //a vector storing the last 2 positions of the robot (y coordinates)
double headingant [2];                                  //a vector storing the last 2 positions of the robot (heding in degrees)
double dleftx[40];                                      //last 40 cells classified as left (x coordinates)
double dlefty[40];                                      //last 40 cells classified as left (y coordinates)
double drightx[40];                                     //last 40 cells classified as right (x coordinates
double drighty[40];                                     //last 40 cells classified as right (y coordinates
double gylant, gxlant, gxrant, gyrant, angleftant=0, angrightant=0, firstangleleft=0, firstangleright=0;
float res=0.020;		                                //Resolution of lane line images meters/px
unsigned int firstL=0, firstR=0;                        // flags to know if first regions Left and right are already classified/identified
//double robotx=0, roboty=0, heading=0, pitch=0,roll=0;
nav_msgs::OccupancyGrid full_map;                           // General Map ...this is "THA MAP"! (includes all the features: obstacles, lane lines, positions...)
nav_msgs::OccupancyGrid unkown_ll;                          //Lane lines map linelanes
nav_msgs::OccupancyGrid l_lane_map;                          //Map of left lanes
nav_msgs::OccupancyGrid r_lane_map;                          //Map of right lanes
nav_msgs::OccupancyGrid posit_map;                          //Positions map - Nobody ever uses this?
nav_msgs::OccupancyGrid pred_l;                          //Predicted left
nav_msgs::OccupancyGrid pred_r;                          //Predicted right
nav_msgs::OccupancyGrid map_region;               //Map of regions to classify left or right lanes (negative for left, positive for right)
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

void convertLocalMaptoLidar(int ix, int iy, int grid_x, int grid_y, int id){
    /*ids
     30 = left lane
     60 = right lane
     45 = unknown lane line
     25 = red flag
     75 = blue flag
     100 = obstacle this one is only used for testing stuff
     */

    float dx=0;
    float dy=0;
    float dt=0;
    float ang;
    unsigned int index;

    dx=(ix-grid_x)*full_map.info.resolution;
    dy=(iy-grid_y)*full_map.info.resolution;
    dt=sqrt((dx*dx)+(dy*dy));
    ang=(atan2(dy,dx));
    if(id==100){ //only for testing
	//ROS_INFO_STREAM("ID = 100");
        index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
        if(index<5000000){ //Check to make sure we're accessing data that its actually possible for the lidar scanner to reach
            localmap1.ranges[index]=dt;
            localmap1.intensities[index]=0;		//left lane
        }
    }
    if(id==30 || id==60){
        index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
        if(index<5000000){ //Check to make sure we're accessing data that its actually possible for the lidar scanner to reach
            localmap1.ranges[index]=dt;
            if(id==30){
                localmap1.intensities[index]=0;
            }
            if(id==60){
                localmap1.intensities[index]=1;
            }//left lane
        }
    }
    if(id==45){
        index=floor((ang-localmap1.angle_min)/localmap1.angle_increment);
        if ((index!=0) && (index<1081)) {
            localmap1.ranges[index]=dt;
            localmap1.intensities[index]=0.5;
        }
        /* I commented out the following two lines b/c im not sure what they do
        if ((index>=180) && (index<300)) {
            x_right[index-180]=dx;
            y_right[index-180]=dy;
        }
        if ((index>780) && (index<=900)) {
            x_left[900-index]=dx;
            y_left[900-index]=dy;
        }
        */
    }
    if(id==25 || id==75){
        index=floor((ang-localmap1.angle_min)/localmap2.angle_increment);
        localmap2.ranges[index]=dt;
        if(id==25) {
            localmap2.intensities[index] = 0;
        }
        if(id==75){
            localmap2.intensities[index] = 1;
        }
    }
}


void laneCallback(const sensor_msgs::Image::ConstPtr& msg) {
    //std_msgs/Header header
    //uint32 height
    //uint32 width
    //string encoding
    //uint8 is_bigendian
    //uint32 step		Full row length in bytes
    //uint8[] data		actual matrix data, size is (step * rows)

    double lx, ly,gyll,gxll,msg_step,msg_height, glob_xleft, glob_yleft;
    double glob_xright, glob_yright, beta, sumldx=0, sumldy=0, sumrdx=0;
    double sumrdy=0, avgly=0, avglx=0, avgry=0, avgrx=0, angleft, angright, probaline;
    msg_step=msg->step;
    msg_height=msg->height;

    unsigned int mapxllind, mapyllind, mapxlind, mapylind, mapxrind, mapyrind;
    for (unsigned int y =msg_height-1; y > 0; y--) {//row
        for (unsigned int x = 0; x < msg_step; x++) { //column
            lx=(msg_height-y)*res;	//distance from robot x
            ly=res*(((msg_step-1)/2)-x);	//distance from robot y
            gxll=(lx*cos(headingant[0]))-(ly*sin(headingant[0]))+robotxant[0]; //dist x from global
            gyll=(lx*sin(headingant[0]))+(ly*cos(headingant[0]))+robotyant[0]; //dist y from global
            mapxllind=floor(gxll/full_map.info.resolution); //global indexes
            mapyllind=floor(gyll/full_map.info.resolution);

            /*ROS_INFO_STREAM("SHIT");
            ROS_INFO_STREAM(robotxant[0]);
            ROS_INFO_STREAM(robotyant[0]);
            ROS_INFO_STREAM(mapxllind);
            ROS_INFO_STREAM(mapyllind);*/
            if ((robotyant[0]!=0) && (robotxant[0]!=0) && mapxllind<full_map.info.width && mapyllind<full_map.info.height && (mapxllind>0) && (mapyllind>0)){
                if (msg->data[MAP_IDX(msg_step, x, y)]==1 && (unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]<100)) {
                    //if img pixel is 1 and unknown lane line map position is < 100; increment cell by 25
                    unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]+=25;	//unknown lane line map
                }
                else if (msg->data[MAP_IDX(msg_step, x, y)]==0 && (unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]<30 && unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]>-120)  ) {
                    //else if img pixel = 0 and unkown lane line map data position is between -120 and 30
                    unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]-=1; //decrease by 1
                }
                if (msg->data[MAP_IDX(msg_step, x, y)]==1 && unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]>=55 && (l_lane_map.data[MAP_IDX(l_lane_map.info.width, mapxllind, mapyllind)]!=100) && (r_lane_map.data[MAP_IDX(r_lane_map.info.width, mapxllind, mapyllind)]!=100)) {
                    //if img pixel is 1 and unkown lane line map data is > 55 and left & right lane map data position is not 100
                    //I believe the following two if statements determine if its a left lane or right lane and extend the seed forward
                    if (map_region.data[MAP_IDX(map_region.info.width, mapxllind, mapyllind)]<-5 || ( (map_prob_left[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]>map_prob_right[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]) && map_prob_left[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]>20)) {
                        //if mapregions map data is < -5 or prob_left_map >prob_right_map and prob_map_left < 20

                        l_lane_map.data[MAP_IDX(l_lane_map.info.width, mapxllind, mapyllind)]=100; //left
                        full_map.data[MAP_IDX(full_map.info.width, mapxllind, mapyllind)]=30;
                        for (unsigned int i=0; i<39; i++) { //this 9 and the ones below should be probably 39. This is setting the seed
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
                                    map_region.data[MAP_IDX(map_region.info.width, mapxllind+i-4, mapyllind+j-4)]=-10; //mapregion is set to -10 for left
                                }
                            }
                        }

                    }

                    if (map_region.data[MAP_IDX(map_region.info.width, mapxllind, mapyllind)]>5) {
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
                                    map_region.data[MAP_IDX(map_region.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
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
                                        map_region.data[MAP_IDX(map_region.info.width, mapxllind+i-4, mapyllind+j-4)]=-10;
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
                                        map_region.data[MAP_IDX(map_region.info.width, mapxllind+i-4, mapyllind+j-4)]=10;
                                    }
                                }
                            }
                        }
                    }
                    unkown_ll.data[MAP_IDX(unkown_ll.info.width, mapxllind, mapyllind)]=120;	//mapa 3 unknown lane
                }
            }  //end of boundaries check)
        } //end of first for
    } //end of second for

    //I belive everything below this is for ray tracing

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
                    probaline=(exp(-(pow((predleft/8),2))))*(exp(-(pow((anguleitor/10),2))));
                    map_prob_left[MAP_IDX(full_map.info.width, mapxlind, mapylind)]+=probaline;
                    pred_l.data[MAP_IDX(pred_l.info.width, mapxlind, mapylind)]+=1;

                }
            }
        }


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
                pred_r.data[MAP_IDX(pred_r.info.width, mapxlind, mapylind)]=100;
            }
        }

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
    //obviously the way this works is it replaces full_map each time so need to put in a way that will
    // not replace it but add to it.
    full_map.info = msg->info;
	ROS_INFO_STREAM(full_map.info.resolution);
    ROS_INFO_STREAM(msg->info.origin.position.x);
	ROS_INFO_STREAM(full_map.info.origin.position.x);
    full_map.header = msg->header;
    full_map.data = msg->data;

    // In gmapping 0 is a free cell, -1 an unknown spot, 100 is an obstacle
    if(setup){ //if this is our first time getting the maps init everything based off of it
        localmap_occup1.info.resolution = msg->info.resolution;
        localmap_occup1.info.width = 50; //Change it 10 meters on each side
        localmap_occup1.info.height = 50;
        localmap_occup1.data.resize(localmap_occup1.info.width * localmap_occup1.info.height);
        for (unsigned int i = 0; i< (localmap_occup1.info.width*localmap_occup1.info.height); i++) {
            localmap_occup1.data[i]=0;
        }

        //MAP 3 (UNKNOWN Lane lines map)
        unkown_ll.info = msg->info;
        unkown_ll.data.resize(unkown_ll.info.width * unkown_ll.info.height);
        //MAP regions  (to classify lane lines)
        map_region.info = msg->info;
        map_region.data.resize(map_region.info.width * map_region.info.height);

        //MAP 4 (Left Lane lines map)
        l_lane_map.info = msg->info;
        l_lane_map.data.resize(l_lane_map.info.width * l_lane_map.info.height);
        //MAP 5 (Right Lane lines map)
        r_lane_map.info = msg->info;
        r_lane_map.data.resize(r_lane_map.info.width * r_lane_map.info.height);

        //MAP 7 (Predicted left)
        pred_l.info = msg->info;
        pred_l.data.resize(pred_l.info.width * pred_l.info.height);
        //MAP 8 (Predicted right)
        pred_r.info =msg->info;
        pred_r.data.resize(pred_r.info.width * pred_r.info.height);

        for (unsigned int i = 0; i< (full_map.info.width*full_map.info.height); i++) {
            unkown_ll.data[i]=0;
            l_lane_map.data[i]=0;
            r_lane_map.data[i]=0;
            pred_l.data[i]=0;
            pred_r.data[i]=0;
            map_region.data[i]=0;

            map_prob_left[i]=0;
            map_prob_right[i]=0; //map prob right or left is sitting directly next to fullmap and overwriting it
        }


        //******INITIALIZE lanelines deltas
        for (unsigned int i = 0; i< 10; i++) {
            dleftx[i]=0;
            dlefty[i]=0;
            drightx[i]=0;
            drighty[i]=0;
        }
        //*******Initialize last 2 poses*******
        for (unsigned int i= 0; i< 2; i++) {
            robotxant[i]=0;
            robotyant[i]=0;
            headingant[i]=0;
        }


        //******INITIALIZE MAPS

        for (unsigned int lmi=0; lmi < 1081; lmi++) {
            localmap1.ranges[lmi]=0;
            localmap2.ranges[lmi]=0;
            localmap1.intensities[lmi]=0.5;
            localmap2.intensities[lmi]=0.5;
        }
	ROS_INFO_STREAM("DONE INIT");
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
    btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
    			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
    			msg->pose.pose.orientation.w);
    btMatrix3x3(q).getEulerYPR(heading, pitch, roll);



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
    ROS_INFO_STREAM(robotx);
    ROS_INFO_STREAM(robotxant[0]);

    //end testing
	
    int iy2, ix2;


    tf::TransformListener listener; //Init the two transform objects needed for getting the transform from map to base link
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) ); //Try to get the transform
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    double map_x = transform.getOrigin().x(); //Get the transformed coordinates
    double map_y = transform.getOrigin().y();
    double map_z = transform.getOrigin().z();
    //ROS_INFO_STREAM("map_X" << transform.getOrigin().x());
    //ROS_INFO_STREAM("map_Y" << transform.getOrigin().y());

    int grid_x;
    int grid_y;
	

    grid_x = (unsigned int)((map_x - full_map.info.origin.position.x) / full_map.info.resolution);
    grid_y = (unsigned int)((map_y - full_map.info.origin.position.y) / full_map.info.resolution);

    robotx = map_x - full_map.info.origin.position.x;
    roboty = map_y - full_map.info.origin.position.x;

    //ROS_INFO_STREAM("gridx " << grid_x);
    //ROS_INFO_STREAM("gridy " << grid_y);
    localmap_occup1.info.origin.position.x=map_x-25*full_map.info.resolution; //Set our position as center of the local map
    localmap_occup1.info.origin.position.y=map_y-25*full_map.info.resolution;
    localmap_occup1.info.origin.position.z=map_z;



    ix2=(grid_x-25);	//in map coordinates
    iy2=(grid_y-25); 	// in map coordinates
    //Everytime you send out the local map in lidar you need to set everyting to 0
    for (unsigned int lmi=0; lmi < 1081; lmi++) {
        localmap1.ranges[lmi]=0;
        localmap2.ranges[lmi]=0;
        localmap1.intensities[lmi]=0.5;
        localmap2.intensities[lmi]=0.5;
    }


    //ROS_INFO_STREAM("ix2 " << ix2);
    //ROS_INFO_STREAM("iy2 " << iy2);

    int i = 0;

    for (int iy=iy2; iy<(iy2+50); iy++) {
        for (int ix=ix2; ix<(ix2+50); ix++) {
            localmap_occup1.data[i] = full_map.data[MAP_IDX(full_map.info.width, ix, iy)];

            if(full_map.data[MAP_IDX(full_map.info.width, ix, iy)]==100) {	//check if there is an obstacle there
                convertLocalMaptoLidar(ix, iy, grid_x, grid_y, 100);
            }
            //Need to add in the other possiblites here like left, right lane, etc once image processing is done
            //as well as the goal

	        ros::Time scan_time = ros::Time::now();
            localmap1.header.stamp = scan_time; //used for publishing local map
            localmap1.header.frame_id = "base_laser_link";
            localmap2.header.stamp = scan_time; //used for publishing local map
            localmap2.header.frame_id = "base_laser_link";
            i++;
        }
    }


}


	
int main(int argc, char **argv) {
    ros::init(argc, argv, "the_mapping");
    ros::NodeHandle n;

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


    //SUBSCRIBE/PUBLISH TO:
    /*

        ros::Publisher localmappingPub = n.advertise<nav_msgs::OccupancyGrid>("localmap_occup1", 1);	//Publish the local map


    ros::Publisher localmappingPub = n.advertise<nav_msgs::OccupancyGrid>("localmap_occup1", 1);	//Publish the local map

    ros::Publisher localmap1Pub = n.advertise<sensor_msgs::LaserScan>("localmap1", 1); //Local1 map (lanelines)
    //ros::Publisher localmap2Pub = n.advertise<sensor_msgs::LaserScan>("localmap2", 1); //Local2 map (flags)
    */
    ros::Publisher mappingPub = n.advertise<nav_msgs::OccupancyGrid>("full_map", 1);	//Publish the Full map
    ros::Subscriber mapSub = n.subscribe("map", 1, retrieveMapCallback); //Subscribe to gmapping map
    ros::Subscriber laneSub = n.subscribe("convertimage", 1, laneCallback);
    ros::Subscriber poseSub = n.subscribe("base_pose_ground_truth", 1, 	extractLocalMap);	//change this so it subscribes to the tf
    ros::Publisher llanemap = n.advertise<nav_msgs::OccupancyGrid>("llanemap", 1);	//Publish the Full map
    ros::Publisher rlanemap = n.advertise<nav_msgs::OccupancyGrid>("rlanemap", 1);	//Publish the local map
    ros::Publisher unknownamp = n.advertise<nav_msgs::OccupancyGrid>("unknownllmap", 1);	//Publish the Full map
    ros::Publisher mapregion = n.advertise<nav_msgs::OccupancyGrid>("mapregion", 1);	//Publish the local map

    ros::Rate rate(45);

    while(ros::ok()) {

        mappingPub.publish(full_map);

        /*
        localmappingPub.publish(localmap_occup1);
        localmap1Pub.publish(localmap1);*/
        //localmap1Pub.publish(localmap1);
        //localmap2Pub.publish(localmap2);


        llanemap.publish(l_lane_map);
        rlanemap.publish(r_lane_map);
        unknownamp.publish(unkown_ll);
        mapregion.publish(map_region);

        ros::spinOnce();
        rate.sleep();
    }
}
