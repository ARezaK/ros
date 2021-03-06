//********************HEADERS NECESSARY TO USE THIS CODE (ALL ARE INCLUDED WITH THE ROS STANDARD PACKAGES)*****************************
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"                      //To receive messages from Lidar
#include "sensor_msgs/Image.h"                          //To receive messages from camera

#include <math.h>                                       //Library to perform math operations
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <stdio.h>                                      //not used

#include "opencv/highgui.h"                             //not used
#include <unistd.h>
#include <tf/transform_listener.h>
//********************************************************************************************************************************************

//*****************************************VARIABLES USED***********************************************************************************


sensor_msgs::Image image_used_for_ll;
sensor_msgs::Image image_used_for_ll2;
//***************************************End of global variables**************************************************************************************

void convertcameraimage(const sensor_msgs::Image::ConstPtr& msg){
    image_used_for_ll.header = msg->header;
    image_used_for_ll.height = msg->height;
    image_used_for_ll.width = msg->width;
    image_used_for_ll.encoding = msg->encoding;
    image_used_for_ll.is_bigendian = msg->is_bigendian;
    image_used_for_ll.step = msg->step;
    image_used_for_ll.data = msg->data;

    //binary image same as above just binary
    image_used_for_ll2.header = msg->header;
    image_used_for_ll2.height = msg->height;
    image_used_for_ll2.width = msg->width;
    image_used_for_ll2.encoding = msg->encoding;
    image_used_for_ll2.is_bigendian = msg->is_bigendian;
    image_used_for_ll2.step = msg->step;
    image_used_for_ll2.data = msg->data;


    int i = 0;
    int size = msg->step*msg->height;
    for(i=0;i<size-1;i++){
        if(msg->data[i]==0){
            image_used_for_ll.data[i]=255;
            image_used_for_ll2.data[i]=255;
        }
        else{
            image_used_for_ll.data[i]=0;
            image_used_for_ll2.data[i]=0;
        }
    }

    for(i=0;i<size-1;i++){
        if(image_used_for_ll.data[i]==255){
            image_used_for_ll.data[i]=1;
        }
        else{
            image_used_for_ll.data[i]=0;
        }
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "afl");
    ros::NodeHandle n;



    //image

    ros::Subscriber imageSub = n.subscribe("image",1,convertcameraimage);
    ros::Publisher imagePub = n.advertise<sensor_msgs::Image>("convertimage", 1); //Local1 map (lanelines)
    ros::Publisher imagePub2 = n.advertise<sensor_msgs::Image>("visualizedimage", 1); //Local1 map (lanelines)

    ros::Rate rate(45);

    while(ros::ok()) {

        imagePub.publish(image_used_for_ll);
        imagePub2.publish(image_used_for_ll2);

        ros::spinOnce();
        rate.sleep();
    }
}
