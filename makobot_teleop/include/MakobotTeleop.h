#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <makobot_teleop/makobot_teleopConfig.h>
#include <makobot_teleop/Arm.h>


class MakobotTeleop {

    public:
        MakobotTeleop();

    private:

        std::vector<int> previous_buttons;


        // Service for arming robot
        ros::ServiceClient arm_client;

        // Subscriber for joystick input
        ros::Subscriber joy_sub;

        // Publisher for thruster msgs
        ros::Publisher thruster_pub;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<makobot_teleop::makobot_teleopConfig> server;
        makobot_teleop::makobot_teleopConfig config;


        // Arms robot by requesting arm from makobot_bridge server
        void arm(bool arm_input, ros::ServiceClient arm_client);

        // Joystick callback function
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

        //
        bool risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index);



};
