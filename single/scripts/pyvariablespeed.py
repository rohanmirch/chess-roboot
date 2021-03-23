#!/usr/bin/env python
#
#   pyvariablespeed.py
#
#   Continually (at 100Hz!) send sinusoidal commands to the robot -
#   but listen for and change the sinusoid frequency.

import sys
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64

from threading import Lock


#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information to the main (timer) loop.
#
offset    = 0.0         # Sinusoid offset (rad)
amplitude = 0.0         # Sinusoid amplitude (rad)
omega      = 0.0        # Sinusoid omega (rad/sec)
phaseshift = 0.0        # Sinusoid phase shift (rad)

t      = 0.0            # Current time (sec)
cmdpos = 0.0            # Current cmd position (rad)
cmdvel = 0.0            # Current cmd velocity (rad/sec)
cmdtor = 0.0            # Current cmd torque (Nm)

# Use a mutex so that parameters are accessed/read/set atomically. 
access_parameters_mutex = Lock()


#
#   Frequency Subscriber Callback
#
#   This message is of type std_msgs::Float64, i.e. it contains only
#   one number.  Use that to set the frequency.  But to keep the
#   position smooth, also adjust the phase!
#
def freqCallback(msg):
    newfrequency = msg.data
    newomega     = 2.0*math.pi * newfrequency

    # Adjust the phaseshift to guaranetee a smooth position transition
    # (yes, the velocity will jump!).  And record the new omega.  Make
    # sure the main code (timer) doesn't access until we're done.
    global phaseshift, omega
    with access_parameters_mutex:
        phaseshift = phaseshift + (omega - newomega) * t
        omega      = newomega;

    # Report.
    rospy.loginfo("Adjusted to %f rad/sec (%f Hz), phaseshift %f" %
                  (newomega, newfrequency, phaseshift))


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pyvariablespeed')

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

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    offset     = msg.position[0]
    amplitude  = math.pi/4.0
    omega      = 2*math.pi * 0.25
    phaseshift = 0.0

    t      = 0.0
    cmdpos = offset
    cmdvel = 0.0
    cmdtor = 0.0

    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    rospy.Subscriber("frequency", Float64, freqCallback)

    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.  Make sure the omega and phaseshift
        # parameters are not changed in the middle of the calculations!
        with access_parameters_mutex:
	    cmdpos = offset + amplitude * math.sin(omega * t + phaseshift)
	    cmdvel =          amplitude * math.cos(omega * t + phaseshift) * omega
	    cmdtor = 0.0

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = cmdpos
        command_msg.velocity[0]  = cmdvel
        command_msg.effort[0]    = cmdtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
