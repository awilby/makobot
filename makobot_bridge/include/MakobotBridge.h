#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <dynamic_reconfigure/server.h>
#include <makobot_teleop/Arm.h>



class MakobotBridge {

    public:
        MakobotBridge();

    private:

        // Node Handle
        ros::NodeHandle n;



        // Service Client for sending mavros commands
        ros::ServiceClient mavros_cmd = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");



        enum {COMPONENT_ARM_DISARM=400};


        // Arm robot by sending request to mavros service
        bool arm(makobot_teleop::Arm::Request&, makobot_teleop::Arm::Response&);

};
