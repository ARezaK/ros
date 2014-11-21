#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <vector>
#include <ctime> // Needed to seed random number generator with a time value
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#define farRange 10


class RandomWalk {
public:
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

    //used to get odometry data
    void GetPoseCallback(const nav_msgs::Odometry::ConstPtr& msg){
        double x;
        double y;
        double w;
        robotx = msg->pose.pose.position.x;
        roboty = msg->pose.pose.position.y;
        x = msg->twist.twist.linear.x;
        y = msg->twist.twist.linear.y;
        w = msg->twist.twist.angular.z;
    };


    enum FSM {
        FSM_MOVE_FORWARD,
        FSM_ROTATE,
        FSM_STOP_TURN,
        FSM_GO_GAP,
        FSM_Obstacle,
        FSM_STOP
    };
    // Tunable parameters
    // TODO: tune parameters as you see fit
    const static double MIN_SCAN_ANGLE_RAD = -90.0 / 180 * M_PI; //adjusted to provide more directionality
    const static double MAX_SCAN_ANGLE_RAD = +90.0 / 180 * M_PI;
    const static float PROXIMITY_RANGE_M = 1.5; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static double FORWARD_SPEED_MPS = 0.4;
    const static double ROTATE_SPEED_RADPS = M_PI/2;    //from M_PI/2
    const static bool on_the_go = true; //switch between on the go and stop and turn mode


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
    ros::Subscriber odomSub; //Subscriber to Odom topic
    unsigned int obstacle_counter;
    int robot_moving;
    unsigned int rotate_now;
    double roation_speed;
    double robotx;
    double roboty;
    double last_pose_x;
    double last_pose_y;
    double time_duration;
    double dist_to_gap;
};

// Constructor definition -- initialize values
RandomWalk::RandomWalk(ros::NodeHandle &nh) {
    fsm = FSM_MOVE_FORWARD;
    robot_moving=0;
    roation_speed=0;
    last_pose_y=roboty;
    rotate_now=0;
    // rotateStartTime and rotateDuration are defined in the Class defn.
    rotateStartTime = ros::Time::now();     //initialize to current time
    rotateDuration = ros::Duration(0.f);    //0.0 duration type float
    rotateDir = 0;


// The ranges element of the Lidar_msg initially has no memory allocated.  The resize function must be
// called before we can copy any data into it.  The 1081 value must be determined based on the scanner setup.
// In this case we have 270 degrees of scanning at 0.25 degree increments or 1080 + 1 for the zero position (
// the classic fence post problem:   x......x......x......x three intervals but 4 posts!
    Lidar_msg.ranges.resize(1081);

    // Initialize random number generator by a call to srand().  Use system time() as seed. This is a standard
    // approach used for the rand() function.
    srand(time(NULL));
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
    odomSub = nh.subscribe("odom", 1, &RandomWalk::GetPoseCallback, this);
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
    unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    double target_orientation;    // orientation to goal
    double current_orientation;   // orientation of robot
    unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    double closestRange = msg->ranges[minIndex];
    float goal_x=-4.5;       //goal x point
    float goal_y=5.5;        //the goal waypoint y value


    for (unsigned int currIndex = (minIndex ); currIndex < (maxIndex); currIndex++) {
        if (msg->ranges[currIndex] <closestRange)
            closestRange = msg->ranges[currIndex];  // find the closest range to the robot
    }

    if(fsm == FSM_MOVE_FORWARD)
    {
        if(closestRange<0.5){  // if there are any obstacles around the robot less than 0.5
            fsm = FSM_Obstacle;  // set the status to obstacle mode
        }
        else{
            obstacle_counter=0;
            for (unsigned int currIndex = (minIndex+250 ); currIndex < (maxIndex-250); currIndex++)
            {
                if (msg->ranges[currIndex] <4){ // if the obstacle ahead is less than 4
                    obstacle_counter++;
                }
            }
            if(obstacle_counter>=101)  {     //If this if statement excutes that means the obstacle density has increased
                // past our threshold
                fsm=FSM_Obstacle; //change status
            }
        }
    }

    if(fsm==FSM_Obstacle)
    {
       unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD-.11 - Lidar_msg.angle_min) / Lidar_msg.angle_increment); //reduce the angle size
       unsigned int  maxIndex = ceil((MAX_SCAN_ANGLE_RAD-.11 - Lidar_msg.angle_min) / Lidar_msg.angle_increment); //reduce the angle size
        closestRange = 4;
        ROS_INFO_STREAM(closestRange);
        rotateDir =-1;
        fsm=FSM_STOP_TURN;
        rotateStartTime = ros::Time::now();
        rotateDuration= ros::Duration(0.5); //for pi/2 rad/s this means 90 degree per second
    }


    if((fsm == FSM_MOVE_FORWARD)||(fsm == FSM_STOP))
    {
        target_orientation=atan2((goal_y-roboty),(goal_x-robotx));

        if(robot_moving<10) {   // Wait for a good amount of time before determining theta
            current_orientation= target_orientation;}
        else{                // we've waited enough now calculate the orientation
            current_orientation=atan2((roboty-last_pose_y),(robotx-last_pose_x));}

        // If we are too far from the goal
        if ((abs(goal_x-robotx)>0.5)||(abs(goal_y-roboty)>0.5))
        {
            // this determines what we have to do to get back on track
            if(target_orientation>current_orientation)
            {
                rotateDir=1;
                roation_speed=(target_orientation-current_orientation)/3.0*ROTATE_SPEED_RADPS;
            }
            else
            {
                roation_speed=(current_orientation-target_orientation)/3.0*ROTATE_SPEED_RADPS;
                rotateDir=-1;
            }
            fsm = FSM_MOVE_FORWARD;
        }
        else
        {
            fsm = FSM_STOP; // We only enter this statement if were not far from the goal anymore
        }
        if(robot_moving<100) robot_moving++;
        last_pose_y=roboty;
        last_pose_x=robotx;
    }
}

void RandomWalk::processSensors() {
    ros::Rate rate(10); // Specify the FSM loop rate in Hz
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
        // TODO: Either call:
        //
        //- move(0, ROTATE_SPEED_RADPS); // Rotate right
        //
        // or
        //
        //- move(FORWARD_SPEED_MPS, 0); // Move forward
        //
        // depending on the FSM state; also change the FSM state when appropriate

        // Analyze the Lidar_msg data and alter the robot state accordingly
        //dfsfd
        if (fsm == FSM_MOVE_FORWARD) {
            move(FORWARD_SPEED_MPS, rotateDir*roation_speed);
        }
        else if(fsm==FSM_STOP_TURN) //you've found the gap now rotate(turn) and enter it
        {   if(on_the_go){
                move(0.1, rotateDir*ROTATE_SPEED_RADPS);
            }
            else{
                move(0,rotateDir*ROTATE_SPEED_RADPS);
            }

            if(ros::Time::now() > (rotateStartTime+rotateDuration))
            {
                rotate_now=0;
                time_duration=dist_to_gap/FORWARD_SPEED_MPS;
                rotateStartTime = ros::Time::now();
                rotateDuration= ros::Duration(time_duration);
                fsm = FSM_GO_GAP;
            }

        }
        else if (fsm == FSM_GO_GAP){ // move the robot to the gap then change state
            move(FORWARD_SPEED_MPS, 0);
            if(ros::Time::now() > (rotateStartTime+rotateDuration))
                fsm = FSM_MOVE_FORWARD;
        }
        else if(fsm==FSM_STOP) //you've reached the goal
        {
            ROS_INFO_STREAM("You have reached the goal");
            move(0, 0);
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

