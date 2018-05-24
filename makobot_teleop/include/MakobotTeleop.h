#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <makobot_teleop/makobot_teleopConfig.h>
//#include <makobot_teleop/Arm.h>


class MakobotTeleop {

    public:
        MakobotTeleop();

    private:

        // CONSTANTS: see https://pixhawk.ethz.ch/mavlink/
        enum {COMPONENT_ARM_DISARM=400};
        enum {FORCE_DISARM = 21196};
        enum {MODE_STABILIZE = 1000, MODE_DEPTH_HOLD = 2000, MODE_MANUAL=1000};  // ppm in uS
        enum {PPS_MIN = 1000, PPS_MAX = 2000};  // ppm in uS
        enum {CAM_TILT_RESET = 1500};  // ppm in uS

        // Keep track of previous buttons pressed
        std::vector<int> previous_buttons;

        // Variables to keep track of current state
        uint16_t mode;
        uint16_t camera_tilt;
        bool initLT;
        bool initRT;

        // ROS Node
        ros::NodeHandle n;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<makobot_teleop::makobot_teleopConfig> server;
        makobot_teleop::makobot_teleopConfig config;

        // Service for arming robot
        ros::ServiceClient arm_client = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

        // Subscriber for joystick input
        ros::Subscriber joy_sub;

        // Publisher for thruster msgs
        ros::Publisher rc_override_pub;


        // Arms robot by requesting arm from mavros command service
        void arm(bool arm_input);

        // Dynamic reconfigure server callback function
        void configCallback(makobot_teleop::makobot_teleopConfig &update, uint32_t level);

        // Joystick callback function
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

        // Get joystick button presses
        bool risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index);

        // Computes the position of joystick for publishing thrust control
        double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

        // Maps axis values to PPM (Pulse Position Modulation) commands
        uint16_t mapToPpm(double in);

};
