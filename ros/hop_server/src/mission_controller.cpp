/*
 * ROS integrated FSM for Hop controls
 * Subscribed: /camera/image, /imu, /safety_onboard, /estop
 * Published: /cmd_vel, /safety_server
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

// Declaring publishers
ros::Publisher cmd_vel_pub;
ros::Publisher safety_pub;

// Set up m_state values
enum State {
    booting,
    standing,
    stable,
    fallen,
    estopped,
    stopping,
    testing
};


class FSM {

    public:  
        // Declaring state tracking
        State m_state;
        State m_prev_state;

        // Declaring constructor for FSM, which takes arg 'ROS node handler'
        FSM(ros::NodeHandle n);

        // Declaring all local msg vars
        geometry_msgs::Twist m_cmd_vel;
        std_msgs::Int8 m_safety_server;
        sensor_msgs::Image m_image;
        sensor_msgs::Imu m_imu;
        std_msgs::Int8 m_safety_onboard;
        std_msgs::Int8 m_estop;

        // Declaring all transformed sensor variables
        std::vector<uint8_t> m_camera_frame;

        // Declaring methods
        void boot();
        void stand();
        void balance();
        void recover();
        void estop();
        void stop();
        void test();

        // Declaring callback methods (subs)
        void cam_callback(sensor_msgs::Image image);
        void imu_callback(sensor_msgs::Imu imu);
        void safety_callback(std_msgs::Int8 safety);
        void estop_callback(std_msgs::Int8 estop);

    private:
        // FSM::boot
        int m_loops_to_spend_in_boot = 20;
        int m_loops_spent_in_boot = 0;
};


/*
* Init finite state machine
*/
FSM::FSM(ros::NodeHandle n) {

    return;
}


/*
 * Boot the robot
 */
void FSM::boot() {

    ROS_INFO("Booting");

    if (m_loops_to_spend_in_boot - m_loops_spent_in_boot <= 0) {

        // Start by standing up
        m_state = standing;
    }
    else {

        m_loops_spent_in_boot++;
    }
}


/*
 * Stand up from a ground state
 */
void FSM::stand() {

    ROS_INFO("Standing");

    m_state = stable;

}


/*
 * Balance in a stable standing state
 */
void FSM::balance() {

    ROS_INFO("Balancing");

    m_state = stable;
}


/*
 * React to a fall
 */
void FSM::recover() {

    ROS_INFO("Recovering");

    m_state = standing;
}


/*
 * Handle an estop state from the safety check or estop topic
 */
void FSM::estop() {

    ROS_INFO("Emergency Stopping");

    m_state = fallen;
}


/*
 * Shut down the robot
 */
void FSM::stop() {

    ROS_INFO("Stopping");

    m_cmd_vel.linear.x = 0.0;
    m_cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(m_cmd_vel);
}


/*
 * Testing state for development
 */
void FSM::test() {

    ROS_INFO("Testing.");

    // Development tests here

    m_state = stopping;
}


/*
 * CALLBACK: Camera image
 */
void FSM::cam_callback(const sensor_msgs::Image msg) {

    m_image = msg;
    m_camera_frame = msg.data;
    // TODO: Convert to opencv image
}


/*
 * CALLBACK: IMU vectors
 */
void FSM::imu_callback(const sensor_msgs::Imu msg) {

    m_imu = msg;
}


/*
 * CALLBACK: Safety Int
 */
void FSM::safety_callback(const std_msgs::Int8 msg) {


    m_safety_onboard = msg;
}


/*
 * CALLBACK: Estop Int
 */
void FSM::estop_callback(const std_msgs::Int8 msg) {

    m_estop = msg;
}


int main(int argc, char **argv) {

    ROS_INFO("Running mission controller main");

    // Initialize ROS node
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n;
    ros::Rate r(10); // rate to loop for (Hertz)

    // Init mission controller
    FSM mission_controller(n);   // init FSM
    mission_controller.m_state = booting; // set initial state

    // ROS Publishers
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    safety_pub = n.advertise<std_msgs::Int8>("/safety_server", 1000);


    // ROS Subscribers
    ros::Subscriber cam_sub = n.subscribe("/camera/image", 1000, &FSM::cam_callback,
            &mission_controller);
    ros::Subscriber scan_sub = n.subscribe("/imu", 1000, &FSM::imu_callback,
            &mission_controller);
    ros::Subscriber safety_sub = n.subscribe("/safety_onboard", 1000, &FSM::safety_callback,
            &mission_controller);
    ros::Subscriber estop_sub = n.subscribe("/estop", 1000, &FSM::estop_callback,
            &mission_controller);

    while (ros::ok()) {

        // FSM decision tree
        switch(mission_controller.m_state) {

            case booting:
                mission_controller.boot();
                break;

            case standing:
                mission_controller.stand();
                break;

            case stable:
                mission_controller.balance();
                break;

            case fallen:
                mission_controller.recover();
                break;

            case estopped:
                mission_controller.estop();
                break;

            case stopping:
                mission_controller.stop();
                break;

            case testing:
                mission_controller.test();
                break;

            default:
                break;
        }

        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Exiting mission_controller");
}