#!/usr/bin/env python
#
#   pywhereisrobot.py
#
#   Simply call the hebiros_node to find the list of actuators.

import sys
import rospy

from sensor_msgs.msg import JointState


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pywhereisrobot')

    # Grab a single message form the feedback/joint_state topic.
    rospy.loginfo("Waiting for topic /hebiros/robot/feedback/joint_state...")
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)

    # Display
    print msg
    rospy.loginfo("The robot is at:")
    for i in range(0, len(msg.position)):
        rospy.loginfo("Joint #%d (name '%s'): position %f" %
                      (i, msg.name[i], msg.position[i]))
