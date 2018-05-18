#include "MakobotTeleop.h"


MakobotTeleop::MakobotTeleop() {

    //ros::NodeHandle n;



    // Subscribe to incoming joystick commands
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, &MakobotTeleop::joy_callback, this);

    // Instantiate service client for arming robot
    //arm_client = n.serviceClient<makobot_teleop::Arm>("makobot_arm");


}


/*
 * Callback function for incoming joystick commands.
 */
void MakobotTeleop::joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {


    // Initialize previous buttons
    if (previous_buttons.size() != joy->buttons.size()) {
         previous_buttons = std::vector<int>(joy->buttons);
    }


    // If robot is armed or disarmed, send service request to makobot_bridge
    if(risingEdge(joy, config.disarm_button)) {
        arm(false);

    } else if(risingEdge(joy, config.arm_button)) {
        arm(true);
    }


}


/*
 *
 */
void MakobotTeleop::arm(bool arm_input) {

    makobot_teleop::Arm srv;
    srv.request.arm = arm_input;

    // If arm/disarm is successful
    if (arm_client.call(srv))
    {

        // If request was for arming
        if (arm_input) {
            ROS_INFO("ROBOT ARMED.");

        // If request was for disarming
        } else {
            ROS_INFO("ROBOT DISARMED.");
        }

    // Otherwise, arm/disarm wasn't successful, log an error
    } else {

        // If request was for arming
        if (arm_input) {
            ROS_ERROR("FAILED TO ARM.");

        // If request was for disarming
        } else {
            ROS_ERROR("FAILED TO DISARM. Warning: Robot is still armed!");
        }

    }

}




/*
 *
 */
bool MakobotTeleop::risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index) {

  return (joy->buttons[index] == 1 && previous_buttons[index] == 0);

}


/*
 * Initializes teleop node.
 */
int main(int argc, char ** argv) {

    // Initialize ROS node
    ros::init(argc, argv, "makobot_teleop");

    MakobotTeleop makobot_teleop;

    ros::spin();

    return 0;

}
