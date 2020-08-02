/*
 * Onboard robot controls
 * - Safety checks
 * - Fast reaction sensing
 * - Sensor message streaming
 * - Execute cmd_vel from controller
 * 
 * Subscribed: /cmd_vel, /safety_server
 * Published: /safety_onboard, /estop
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

#include <stdlib.h>


// Declaring publishers
ros::Publisher estop_pub;
ros::Publisher safety_pub;


/*
 * Make sure safety checks are happening
 */
void check_timeout() {

}


/*
 * CALLBACK: Command velocity Twist
 */
void cmd_callback(const geometry_msgs::Twist msg) {

}


/*
 * CALLBACK: Safety Int
 */
void safety_callback(const std_msgs::Int8 msg) {

    // Check server computed echo

    // Compute response to server prompt

    // Generate new prompt
}


int main(int argc, char**argv) {

    ros::NodeHandle nh;
    ros::init(argc, argv, "hindbrain");
	
    // ROS Publishers
    safety_pub = nh.advertise<std_msgs::Int8>("/safety_onboard", 1000);
    estop_pub = nh.advertise<std_msgs::Int8>("/estop", 1000);

    // ROS Subscribers
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 1000, cmd_callback);
    ros::Subscriber safety_sub = nh.subscribe("/safety_server", 1000, safety_callback);

	ros::Rate r(10);
	while(ros::ok()) {
        check_timeout();

		r.sleep();
        ros::spinOnce();
	}
}