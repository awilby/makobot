#include "MakobotTeleop.h"


MakobotTeleop::MakobotTeleop() {

    // Subscribe to incoming joystick commands
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, &MakobotTeleop::joy_callback, this);

    // Publish thrust commands on mavros rc_override
    rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

}


/*
 * Callback function for incoming joystick commands.
 */
void MakobotTeleop::joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {


    // Initialize previous buttons
    if (previous_buttons.size() != joy->buttons.size()) {
         previous_buttons = std::vector<int>(joy->buttons);
    }


    // ARMING
    if (risingEdge(joy, config.disarm_button)) {
        arm(false);

    } else if(risingEdge(joy, config.arm_button)) {
        arm(true);
    }

    // MODE SWITCHING: Manual, stabilize, depth hold
    if (risingEdge(joy, config.stabilize_button)) {
        mode = MODE_STABILIZE;

    } else if (risingEdge(joy, config.alt_hold_button)) {
        mode = MODE_ALT_HOLD;
    }


    // CAMERA TILT
    if (risingEdge(joy, config.cam_tilt_reset)) {
         camera_tilt = CAM_TILT_RESET;

    } else if (risingEdge(joy, config.cam_tilt_up)) {
        camera_tilt = camera_tilt + config.cam_tilt_step;

        if (camera_tilt > PPS_MAX) {
            camera_tilt = PPS_MAX;
        }
    }


    // Remember current button states for future comparison
    previous_buttons = std::vector<int>(joy->buttons);


    // RC OVERRIDE MESSAGE
    mavros_msgs::OverrideRCIn msg;

    // THRUSTER CONTROL: forward, strafe, throttle
    msg.channels[5] = mapToPpm(config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo)); // forward  (x)
    msg.channels[6] = mapToPpm(config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo)); // strafe   (y)
    msg.channels[2] = mapToPpm(config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo)); // throttle (z)

    // THRUSTER CONTROL: roll, pitch, yaw
    msg.channels[1] = mapToPpm(config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo)); // roll     (wx)
    msg.channels[0] = mapToPpm(config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo)); // pitch    (wy)
    msg.channels[3] = mapToPpm(config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo)); // yaw      (wz)

    // MODE AND CAMERA CONTROL
    msg.channels[4] = mode; // mode
    msg.channels[7] = camera_tilt; // camera tilt

    rc_override_pub.publish(msg);

}


/*
 * Gets current axis value from joystick and returns exponentially scaled value.
 * Input:
 *     joy: incoming value from joystick driver
 *     index: which joystick is command issued on
 *     expo: exponential scaling factor
 * Output:
 *     exponentially-scaled value representing joystick axis position
 */
double MakobotTeleop::computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo) {

    // Return 0 if axis index is invalid
    if(index < 0 || index>= joy->axes.size()) {
        return 0.0;
    }

    // grab axis value
    double value;

    // The joystick driver initializes all values to 0.0, however, the triggers
    // physically spring back to 1.0 - let's account for this here
    if(index == 6) {

        double lt = joy->axes[2];
        double rt = joy->axes[5];

        if(lt < -0.01 || lt > 0.01) initLT = true;
        else if(!initLT) lt = 1.0;

        if(rt < -0.01 || rt > 0.01) initRT = true;
        else if(!initRT) rt = 1.0;

        // this is the trigger pair pseudo axis (LT-RT; pressing RT results in a positive number)
        value = (lt - rt) / 2.0;

    } else {
        value = joy->axes[index];

    }

    // apply exponential scaling
    return expo * pow(value, 5) + (1.0 - expo) * value;

}


/*
 * Maps joystick positions to PPM (Pulse Position Modulation) values in microseconds.
 * Input:
 *     in: value from -1 to 1 representing joystick position
 * Output:
 *     out: value from 1000 to 2000 (microseconds) for PPM
 */
uint16_t MakobotTeleop::mapToPpm(double in) {

    // Convert joystick value to PPM
    uint16_t out = 1000 + (in + 1.0) * 500;

    // Limit output value to correct PPM ranges
    if(out > 2000) {
        return 2000;
    } else if(out < 1000) {
        return 1000;
    } else {
        return out;
    }
}


/*
 * Sends a request to makobot_bridge service to arm the robot.
 */
/*void MakobotTeleop::arm(bool arm_input) {

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

}*/

void MakobotTeleop::arm(bool arm_input) {

    // Set up service request from mavros command service
    mavros_msgs::CommandLong srv;
    srv.request.command = COMPONENT_ARM_DISARM;
    srv.request.param1 = (arm_input ? 1 : 0);
    srv.request.param2 = FORCE_DISARM;

    // Send request to service
    if (arm_client.call(srv)) {
        ROS_INFO(arm_input ? "Armed" : "Disarmed" );
    } else {
        ROS_ERROR("Failed to update arming");
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
