#include "PixhawkBridge.h"


/*
 * Constructor for PixhawkBridge. Sets up publishers and subscribers to required topics from mavros
 * and waits for connection to FCU.
 */
PixhawkBridge::PixhawkBridge() {

    // Set up publishers and subscribers
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &PixhawkBridge::state_callback, this);
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    // Setpoint publishing rate must be faster than 20Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connection to Pixhawk established.");

}

/*
 * Sends arm/disarm request to Pixhawk.
 * Input: boolean value indicating whether to arm or disarm
 *     TRUE value means arm robot
 *     FALSE value means disarm robot
 */
void PixhawkBridge::arm(bool arm_input) {

    mavros_msgs::CommandBool arm_cmd;

    arm_cmd.request.value = arm_input;


    // Call arming service
    if (arm_client.call(arm_cmd) && arm_cmd.response.success) {

        // If request was for arming)
        if (arm_input) {
            ROS_INFO("Robot armed.");

        // If request was for disarming
        } else {
            ROS_INFO("Robot disarmed.");

        }

    // Otherwise arming wasn't successful, log an error
    } else {

        // If request was for arming
        if (arm_input) {
            ROS_ERROR("Failed to arm robot.");

        // If request was for disarming
        } else {
            ROS_ERROR("FAILED TO DISARM. WARNING: Robot is still armed!");
        }

    }

}

/*
 * Callback for getting current state from FCU.
 */
void PixhawkBridge::state_callback(const mavros_msgs::State::ConstPtr& state_msg) {

    current_state = *state_msg;

}







/*
 * Initializes pixhawk_bridge_node for communication with Pixhawk and
 * all sensors and actuators attached to Pixhawk.
 */
int main(int argc, char ** argv) {

    // Initialize pixhawk_bridge_node
    ros::init(argc, argv, "makobot_pixhawk_bridge");

    ROS_INFO("Connecting to Pixhawk controller...");

    PixhawkBridge pixhawk_bridge;

    ros::spin();

    return 0;

}