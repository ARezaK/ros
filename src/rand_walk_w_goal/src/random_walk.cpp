#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <vector>
#include <ctime> // Needed to seed random number generator with a time value

class RandomWalk {
public:
    //Last
    // Establish a prototype for the constructor for the RandomWalk class.  Note that we pass the node
    // handle as reference which is necessary for the advertise and subscribe methods below.
    RandomWalk(ros::NodeHandle &nh);

    // Send vehicle commands -- Method prototype
    void move(double linearVelMPS, double angularVelRadPS);

    // Process the incoming laser scan message -- Method prototype
    void commandCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    // Prototype for the Main FSM loop for ensuring that ROS messages are
    // processed in a timely manner, and also for sending
    // velocity controls to the simulated robot based on the FSM state
    void processSensors();


    enum FSM {
        FSM_MOVE_FORWARD, FSM_ROTATE, FSM_CHANGE
    };
    // Tunable parameters
    // TODO: tune parameters as you see fit
    const static double MIN_SCAN_ANGLE_RAD = -90.0 / 180 * M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +90.0 / 180 * M_PI;
    const static float PROXIMITY_RANGE_M = 1.0; // Should be smaller than sensor_msgs::LaserScan::range_max
// Create a LaserScan message object that can be used to copy the laser callback message for later processing
    sensor_msgs::LaserScan Lidar_msg;
// Note:  You cannot execute the resize operation in the class definition.  This amounts to a method
//   execution since we are calling the resize function.  This should be done in the class constructor, or in the
//   callback.  In the constructor however you do not know the scan size and must use a "magic number" to specify
//   the size of the laser scan array.  In the callback however, the resize command would end up being executed
//   repeatedly...
//    Lidar_msg.ranges.resize(1081);

// Use the protected modifier to indicate that objects are visible to this class and friend classes as well
protected:
    // Instantiate objects that are later assigned values in the methods associated with the randomwalk Class
    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
    enum FSM fsm; // Finite state machine for the random walk algorithm
    ros::Time rotateStartTime; // Start time of the rotation
    ros::Duration rotateDuration; // Duration of the rotation
    int rotateDir;
    double ROTATE_SPEED_RADPS;
    double FORWARD_SPEED_MPS;

};

// Constructor definition -- intialize values
RandomWalk::RandomWalk(ros::NodeHandle &nh) {
    fsm = FSM_MOVE_FORWARD;
    // rotateStartTime and rotateDuration are defined in the Class defn.
    rotateStartTime = ros::Time::now();     //initialize to current time
    rotateDuration = ros::Duration(0.f);    //0.0 duration type float
    rotateDir = 0;
    ROTATE_SPEED_RADPS = M_PI / 10;
    FORWARD_SPEED_MPS = 1.0;


// The ranges element of the Lidar_msg initially has no memory allocated.  The resize function must be
// called before we can copy any data into it.  The 1081 value must be determined based on the scanner setup.
// In this case we have 270 degrees of scanning at 0.25 degree increments or 1080 + 1 for the zero position (
// the classic fence post problem:   x......x......x......x three intervals but 4 posts!
	Lidar_msg.ranges.resize(1081);

    // Initialize random number generator by a call to srand().  Use system time() as seed. This is a standard
    // approach used for the rand() function.
    srand(time(NULL));
    // Advertise a new publisher for the simulated robot's velocity command topic
    // (the second argument indicates that if multiple command messages are in
    // the queue to be sent, only the last command will be sent).  The roscpp node handle (nh) advertise method
    // returns a publisher object which is assigned to the commandPub object instantiated in the RandomWalk class.
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribe to the simulated robot's laser scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    // Since the callback is defined within a class, the ROS system needs to know where it is from when the roscpp
    // handler processes the message data and the associated callbacks.  The "this" pointer carries the address
    // of the method's parent object or class.  The roscpp node handle (nh) subscribe method
    // returns a publisher object which is assigned to the laserSub object instantiated in the RandomWalk class.
    // base_scan is the topic name produced by stage simulation environment for the Lidar sensor.
    laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
}

// Send a velocity command
void RandomWalk::move(double linearVelMPS, double angularVelRadPS) {
    // The default constructor will set all commands to 0.  Recall that this msg is local to this function and
    // thus does not conflict with msg objects in other methods.
    geometry_msgs::Twist msg;
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    // Set the Twist message forward and angular speed values and publish the method on the cmd_vel topic
    commandPub.publish(msg);
}

void RandomWalk::commandCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	double N;
// Copy data from callback message into our Lidar_msg so that we can process the data in
// our main processing loop.  The callbacks are used then to capture and possibly convert
// data from multiple sensors, and all interpretation can then be done in one coherent process.
// Here we have copied all the data except the complete header and the intensities[] array.
	Lidar_msg.header.stamp = msg->header.stamp;	// System time stamp for the lidar scan
	Lidar_msg.angle_min = msg->angle_min;
	Lidar_msg.angle_max = msg->angle_max;
	Lidar_msg.angle_increment = msg->angle_increment;
	Lidar_msg.time_increment = msg->time_increment;
	Lidar_msg.scan_time = msg->scan_time;
	Lidar_msg.range_min = msg->range_min;
	Lidar_msg.range_max = msg->range_max;

	N= 1+ floor((msg->angle_max - msg->angle_min) / msg->angle_increment);
	//ROS_INFO_STREAM("Size of Range array: " << N); //1081
	Lidar_msg.ranges=msg->ranges;
// Loop to print values in ranges array for testing purposes
//	for (unsigned int currIndex = 1; currIndex <= 1081; currIndex++) {
//		data[currIndex]=msg->ranges[currIndex];
//		ROS_INFO_STREAM("Range value: " << Lidar_msg.ranges[currIndex]);
//	}
}

void RandomWalk::processSensors() {
    ros::Rate rate(10); // Specify the FSM loop rate in Hz
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
        ROS_INFO_STREAM("START");
        //roslaunch rand_walk_w_goal random_walk.launch
        for (unsigned int i=0 ; i <250; i++) { //Check the right side of the robot. 250 b/c we want the right side up to ~30 degree angle
                if(Lidar_msg.ranges[i]  < 1.4){ //If there is an object within 1.4 meters angle the robot towards the left
                    //rotate to the left
                    rotateDir = 1;
                    move(FORWARD_SPEED_MPS, rotateDir * ROTATE_SPEED_RADPS);
                }
                if(Lidar_msg.ranges[i] > 1.5){ //"" angle the robot towards the right
                    //rotate to the right
                    rotateDir = -1;
                    move(FORWARD_SPEED_MPS, rotateDir * ROTATE_SPEED_RADPS);
                }
        }
        //******************************************************//
        float closestRange = 2;
        for (unsigned int i=490 ; i <580; i++) { //Check the front of the robot
            if (Lidar_msg.ranges[i] < closestRange) {
                closestRange = Lidar_msg.ranges[i]; // if there is any ranges less than 2 meters set that as closest range
                }
        }
        ROS_INFO_STREAM(closestRange);

        if(closestRange < 2){
            if(FORWARD_SPEED_MPS >0.10){
            FORWARD_SPEED_MPS = FORWARD_SPEED_MPS - 0.04; //incrementally slow down but never stop (hence the > 0.1)

            }
            rotateDir = 1; //Rotate away from impendeing object
            move(FORWARD_SPEED_MPS, rotateDir * ROTATE_SPEED_RADPS);
            ROTATE_SPEED_RADPS = ROTATE_SPEED_RADPS + 0.35; //Incrementally update your rotate speed
        }

        if(closestRange==2){ //If closest range is back to 2 set everything back to default values
            ROTATE_SPEED_RADPS = M_PI / 2.5;
            FORWARD_SPEED_MPS = 8.0;
        }

        //******************************************************//
        //check for worst case scenario
        if(Lidar_msg.ranges[540] < 0.5 && Lidar_msg.ranges[10] < 5){ //Check if an object is directly in front of this and an object is to the right of us
            ROS_INFO_STREAM("WORSTCASE");
            FORWARD_SPEED_MPS = 0.3; // SLOW DOWN
            rotateDir = 1;
            // rotate for 1.5 seconds
            int timer = 1;
            while(timer<500){
                move(FORWARD_SPEED_MPS,rotateDir * ROTATE_SPEED_RADPS);
                timer++;
            }
        }

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
    ros::NodeHandle n;
    RandomWalk walker(n); // Create new random walk object
    walker.processSensors(); // Execute FSM loop
    return 0;
}

