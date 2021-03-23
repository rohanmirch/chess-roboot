#!/usr/bin/env python
#
#   pysendcommands.py
#
#   Continually (at 100Hz!) send commands to the robot
#   (in hebiros_node).

import sys
import rospy
import math

from sensor_msgs.msg import JointState


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pysendcommands')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)

    command_msg = JointState()
    command_msg.name.append('Photon/3')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)

    
    # Create a servo loop at 100Hz.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()

    # Run the servo loop until shutdown.
    rospy.loginfo("Running the servo loop with dt %f" % dt)
             
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.
        cmdpos = math.pi/4.0 * math.sin(2.0*math.pi * 0.25 * t)
        cmdvel = math.pi/4.0 * math.cos(2.0*math.pi * 0.25 * t) * 2.0*math.pi * 0.25
        cmdtor = 0.0

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = cmdpos
        command_msg.velocity[0]  = cmdvel
        command_msg.effort[0]    = cmdtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
