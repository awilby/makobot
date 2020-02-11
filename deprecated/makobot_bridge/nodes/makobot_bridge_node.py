#!/usr/bin/env python
import rospy


def main():
    """ Starts makobot_bridge node and instantiates MakobotBridge object for communicating with Makobot."""

    # Initialize ROS node
    rospy.init_node('makobot_bridge', anonymous=False, log_level=rospy.DEBUG)

    # Instantiate MakobotBridge
    MakobotBridge()



if __name__ == "__main__":

    try:
        rospy.loginfo("Starting Makobot...")
        main()
        rospy.loginfo("Makobot Bridge successfully started.")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Makobot node terminated.")
