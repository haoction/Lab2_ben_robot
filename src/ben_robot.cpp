// Zehao-Automatic obstacle avoidance robot

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

const double MIN_SAFE_DISTANCE = 0.2; // set alarm if anything is within 0.4m of the front of robot, the safe distance must greater than the radium of robot.

// these values to be set within the laser callback
float ping_dist_in_front_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_ = -1; // NOT real; callback will have to find this
int ping_index_right_ = -1;//serial number of laser-right 
int ping_index_left_ = -1;//serial number of laser-left 
int index_check = -1;//serial number of laser
double angle_check = 0.0;//angle to check
double angle_min_ = 0.0;//the minimum angle of laser
double angle_max_ = 0.0;//the maximum angle of laser
double angle_increment_ = 0.0;//precision of laser
double range_min_ = 0.0;//the minimum distance of laser 
double range_max_ = 0.0;//the maximum distance of laser
double range_check = 0.0;//the distance of laser_checking
bool laser_alarm_ = false;
float R = 0.15; // the radius of the robot
#define PI 3.14159265
char *topic_name;


geometry_msgs::Twist twist_cmd;
geometry_msgs::Twist twist_output;
//This is the message type required to send twist commands to STDR


ros::Publisher vel_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	if (ping_index_ < 0) {
		//for first message received, set up the desired index of Laser range to eval
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;
		// what is the index of the ping that is straight ahead?
		// BETTER would be to use transforms, which would reference how the LIDAR is mounted;
		// but this will do for simple illustration
		ping_index_ = (int)((0.0 - angle_min_) / angle_increment_);//get the index number of the front, 0 degree
		ping_index_right_ = (int)((-PI / 2 - angle_min_) / angle_increment_);//get the index number of right, -90 degree
		ping_index_left_ = (int)((PI / 2 - angle_min_) / angle_increment_);//get the index number of left, 90 degree
		ROS_INFO("LIDAR setup: ping_index = %d", ping_index_);

	}

	ping_dist_in_front_ = laser_scan.ranges[ping_index_];//get the distance of the front
	index_check = ping_index_right_;//Initialization the index at the -90 degree 
 	angle_check = -angle_min_ -index_check * angle_increment_;//get the angle of index_check
	laser_alarm_ = false;

	while (index_check < ping_index_left_ + 1)
	{
		range_check = laser_scan.ranges[index_check];//get the ditance
		//there is three situations, judge for all situations 
                if (angle_check > atan(R/MIN_SAFE_DISTANCE)) { //angle in the right area
			if (range_check < R / sin(angle_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		if ((angle_check < atan(MIN_SAFE_DISTANCE / R)) && (angle_check > - atan(R/MIN_SAFE_DISTANCE))) {//angle in the front area
			if (range_check < MIN_SAFE_DISTANCE/cos(angle_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		if (angle_check < - atan(R/MIN_SAFE_DISTANCE )) {//angle in the left area
			if (range_check < R / sin(- angle_check)) {
				laser_alarm_ = true;
				break;
			}
		}
		
                index_check++;//index number plus 1
		angle_check = angle_check + angle_increment_;//get the new angle
                //traverse
	}
	twist_output = twist_cmd;//output hte twist_cmd
	if (laser_alarm_) {
               
		ROS_WARN("DANGER, WILL ROBINSON!!");
                ROS_INFO("Distance to barrier = %f", range_check);
                ROS_INFO("Angle to barrier = %f", angle_check);
                //show dangerous distance and angle when the distance is with safedistance  
		twist_output.linear.x = (twist_output.linear.x) / 10;//can change the velocity to get different performance
		if (angle_check > 0) {
			twist_output.angular.z = 1 + twist_output.angular.z;//turn left
		}
		else {
			twist_output.angular.z = -1 + twist_output.angular.z;//turn right
		}
	}
        //decrease the speed forward velocity when in dangerous distance and turn to the safe angle
	
	vel_publisher_.publish(twist_output);

}
void des_vel_Callback(const geometry_msgs::Twist& cmd) {//get the messages of geometry_msgs/Twist and cmd
        
	twist_cmd = cmd;//transfer
        ROS_INFO("Velocity = %f", twist_cmd);
     }



int main(int argc, char **argv) {
	ros::init(argc, argv, "ben_robot"); //name this node
	// Process command line parameter looking for a -n string name
	// and should be placed after the ros::init() invocation.
	// rosrun <package_name> <executable_name> -n <new_name>
	// or
	// rosrun subscriber_package subscriber_node -n alternate_topic
	int opt;
	while ((opt = getopt(argc, (argv), "n:")) != -1) {
  		switch (opt) {
    			case 'n':
      				topic_name = optarg;
      				break;
    			default:
      				printf("The -%c is not a recognized parameter\n", opt);
      				break; 
  		}
	}
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);//create a publisher object and have it subscribe to the lidar topic
	vel_publisher_ = pub; // let's make this global, so callback can use it
	ros::Subscriber des_vel = nh.subscribe("des_vel", 1, des_vel_Callback);
	ros::Subscriber lidar_subscriber = nh.subscribe(topic_name, 1, laserCallback);//create a Subscriber object and have it subscribe to the lidar topic
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}
