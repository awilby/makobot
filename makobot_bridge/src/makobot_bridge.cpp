/*
 * TODO
 * Author: Antonella Wilby <awilby@ucsd.edu>
 */

#include "MakobotBridge.h"

MakobotBridge::MakobotBridge() {

    // Instantiate ROS node handle
    //ros::NodeHandle n;

    // Mavros service for sending arm command


}


/*
 * Arm robot by sending request to mavros service.
 * Returns true if arm/disarm successful, false otherwise.
 */
bool MakobotBridge::arm(makobot_teleop::Arm::Request &req,
                        makobot_teleop::Arm::Response &res) {

    bool arm = req.arm;

    // Generate mavros service request
    mavros_msgs::CommandLong srv;
    srv.request.command = COMPONENT_ARM_DISARM;
    srv.request.param1 = (arm ? 1 : 0);
    srv.request.param2 = 21196;

    // Send arm/disarm command to mavros
    // If arm/disarm was successful
    if(mavros_cmd.call(srv)) {
        ROS_INFO(arm ? "Armed" : "Disarmed");

        return true;

    // If arm/disarm unsuccessful
    } else {
        ROS_ERROR("Failed to update arm state");
        return false;
    }

}



/*
 * Starts node for communicating with MakoBot.
 * Starts thruster controllers, sensor interfaces, etc.
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "makobot_bridge");

    MakobotBridge makobot_bridge;

    ros::spin();

    return 0;

}