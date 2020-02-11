#!/usr/bin/env python
import rospy
from pymavlink import mavutil
from mavros_msgs.msg import CommandLong, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

class MakobotBridge(object):
    """ Bridge between Makobot and mavlink interface.
    """

    def __init__(self, device='udp:192.168.2.1', baud=115200):
        """ Initialize MakobotBridge object
            Args:
                device (str, optional): mavprox udp port
                baud (int, optional): baudrate
        """


        self.mavros_arm_service = '/mavros/cmd/arming'
        self.mavros_mode_service = '/mavros/set_mode'


        self.mavros_battery = '/mavros/battery'

        # Subscribe to mavros topics
        rospy.Subscriber("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)
        rospy.Subscriber("/mavros/battery", sensor_msgs.msg.BatteryState)


    def publish_odometry():
        """..."""

        pass


    def arm(self, do_arm):
        """ Send arm/disarm commands to pixhawk through mavros service.
            Args:
                do_arm: bool where true = arm the robot and false = disarm the robot
        """

        # Send arm message to mavros service
        rospy.wait_for_service(self.mavros_arm_service)

        try:
            self.arm_service = rospy.ServiceProxy(self.mavros_arm_service, CommandBool)
            self.arm_service(do_arm)

        except rospy.ServiceException, e:
            if (do_arm):
                rospy.loginfo("Failed to arm robot.")
            else:
                rospy.loginfo("Failed to disarm robot.")


    def set_mode(self):
        """Updates flight mode by sending command to mavros service."""

        # Set to guided mode
        rospy.wait_for_service(self.mavros_mode_service)
        mode_service = rospy.ServiceProxy(self.mavros_mode_service, SetMode)
        mode_service(custom_mode='MANUAL')


    def shutdown(self):
        """Sends disarm command to mavros service for clean shutdown."""

        rospy.loginfo("Disarming robot and shutting down...")
        self.arm(False)
        rospy.signal_shutdown("Shut down robot.")
