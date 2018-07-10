#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class PixhawkBridge {

    public:
        PixhawkBridge();


        // Arm pixhawk
        void arm(bool arm_input);

        // Get flight mode from Pixhawk


    private:
        // CONSTANTS: see https://pixhawk.ethz.ch/mavlink/
        enum {COMPONENT_ARM_DISARM=400};
        enum {FORCE_DISARM = 21196};
        enum Mode {MODE_STABILIZE = 1000, MODE_DEPTH_HOLD = 2000, MODE_MANUAL=1000};  // ppm in uS
        enum PPS {PPS_MIN = 1000, PPS_MAX = 2000};  // ppm in uS
        enum {CAM_TILT_RESET = 1500};  // ppm in uS

        ros::NodeHandle nh_;

        // Current state of FCU
        mavros_msgs::State current_state;

        // Subscriber for getting state information from pixhawk via mavros
        ros::Subscriber state_sub;

        // Publisher for local position setpoint
        ros::Publisher local_pos_pub;

        // Service for arming Pixhawk
        ros::ServiceClient arm_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

        // Service for setting flight mode
        ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // State callback function: gets current state from FCU
        void state_callback(const mavros_msgs::State::ConstPtr& state_msg);



};