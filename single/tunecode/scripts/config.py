#!/usr/bin/env python
#
#   config.py
#
#   Start a session, defining, checking, and configuring the robot.
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
    rospy.init_node('config')

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

    # Configure the base actuator.  Note the defaults:
    # X5-4:  hebi_set_gains('family/name', 10, 0.05, 1200.0)   Kplim ~ 70
    # X5-9:  hebi_set_gains('family/name', 15, 0.05, 1200.0)   Kplim ~130
    # X8-9:  hebi_set_gains('family/name',  5, 0.03, 1200.0)   Kplim ~ 70
    # X8-16: hebi_set_gains('family/name',  5, 0.03, 1200.0)   Kplim ~170
    #hebi_set_gains(fullnames[0], 10, 0.05, 1200.0)
    #hebi_set_gains(fullnames[1], 10, 0.05, 1200.0)
    #hebi_set_gains(fullnames[2], 10, 0.05, 1200.0)
    hebi_set_gains(fullnames[0], 10, 0.05, 1200.0)
    hebi_set_gains(fullnames[1], 60, 0.4,  400.0)
    hebi_set_gains(fullnames[2], 10, 0.05, 1200.0)

    # Potentially adjust the force gains.  Careful.  Defaults:
    # X5-4:  hebi_set_gains('family/name', 0.25, 0.001 , 1636.0)
    # X5-9:  hebi_set_gains('family/name', 0.25, 0.001 , 1636.0)
    # X8-9:  hebi_set_gains('family/name', 0.1 , 0.0001, 1636.0)
    # X8-16: hebi_set_gains('family/name', 0.1 , 0.0001, 1636.0)
    hebi_set_forcegains(fullnames[1], 0.25, 0.002, 1636)
