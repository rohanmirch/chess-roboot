#!/usr/bin/env python
#
#   watch.py
#
#   Simply watch the robot...
#
import rospy

from robotutils import *


#
#   Main Routine
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('watch')

    # Use a 5Hz servo to print until shutdown.
    servo = rospy.Rate(5)
    while not rospy.is_shutdown():
        show_joints()
        servo.sleep()
