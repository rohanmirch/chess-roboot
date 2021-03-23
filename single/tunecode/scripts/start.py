#!/usr/bin/env python
#
#   start.py
#
#   Start the session, defining and checking the robot.
#
import rospy

from robotutils import *


#
#   Robot Definition
#
family = 'Dwarfs'
names  = ['Doc', 'Sleepy', 'Grumpy']

fullnames = [family+'/'+name for name in names]


#
#   Main Routine
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('start')

    # Do some ping-ing.  For some teams (going through the switch?)
    # actuators aren't visible with a ping?  These are my addresses.
    ping('10.10.10.90')
    ping('10.10.10.91')
    ping('10.10.10.92')

    # Assert the HEBI node is running.
    hebi_assert_node()

    # Check the HEBI's actuator list.
    hebi_show_actuators()

    # Define/check the robot joints.
    hebi_define_robot(family, names)

    # Check the joints.
    show_joints()
