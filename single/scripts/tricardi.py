#!/usr/bin/env python
#
#   triangle.py
#
#   Command a triangle wave around a nominal position.
#
import math
import rospy

from robotutils import *
from gravity    import *


#
#   Robot Definition
#
family = 'Photon'
names  = ['5', '6', '1', '2', '3']

fullnames = [family+'/'+name for name in names]


#
#   Main Routine
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('triangle')

    # Do some ping-ing.  For some teams (going through the switch?)
    # actuators aren't visible with a ping?  These are my addresses.
    #ping('10.10.10.90')
    #ping('10.10.10.91')
    #ping('10.10.10.92')

    # Assert the HEBI node is running.
    hebi_assert_node()

    # Check the HEBI's actuator list.
    hebi_show_actuators()

    # Define/check the robot joints.
    hebi_define_robot(family, names)

    # Check the joints.
    show_joints()
    
    # Grab the initial position/velocity/effort.
    (initpos, initvel, initeff) = get_joints()

    # Define a command message, which will encode (save) the state of
    # the current commands.  Start at the current values.
    cmdmsg              = JointState()
    cmdmsg.header.stamp = rospy.Time.now()
    cmdmsg.name         = fullnames
    cmdmsg.position     = initpos
    cmdmsg.velocity     = initvel
    cmdmsg.effort       = initeff


    # Create a publisher to send commands to the robot.  Allow initial
    # time for the subscriber to connect avoiding any missed commands.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state',
                          JointState, queue_size=10)
    rospy.sleep(0.4)
    pub.publish(cmdmsg)


    # Gradually change the effort to the gravity model.
    blocking_rampeffort(pub, cmdmsg, gravity(cmdmsg.position))

    # Then move from the current (initial) to the nominal.
    blocking_moveto(pub, cmdmsg, [0.4, -0*math.pi/2, 0*math.pi/2, 0, 0], gravity)

    # Finally (continually) output the triangle wave.
    blocking_trianglewave(pub, cmdmsg, 1, 0.1, 2.0, gravity)
