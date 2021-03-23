#!/usr/bin/env python
#
#   pymovetoexplicit.py
#
#   Continually (at 100Hz!) send commands to the robot, providing
#   explicit moves to goal locations - using a cubic spline.
#
import sys
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64
from opencv_apps.msg import FaceArrayStamped

from threading import Lock


#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information to the main (timer) loop.
#
t0 = 0.0		# Cubic spline start time
tf = 0.0		# Cubic spline final time
a  = 0.0		# Cubic spline t^0 parameter
b  = 0.0		# Cubic spline t^1 parameter
c  = 0.0		# Cubic spline t^2 parameter
d  = 0.0		# Cubic spline t^3 parameter
                                                        
t      = 0.0            # Current time (sec)
cmdpos = 0.0            # Current cmd position (rad)
cmdvel = 0.0            # Current cmd velocity (rad/sec)
cmdtor = 0.0            # Current cmd torque (Nm)

# Use a mutex so that parameters are accessed/read/set atomically. 
access_parameters_mutex = Lock()


#
#   Reset the spline parameters.
#
#   Compute and load the spline parameters
#
def setspline(goalpos, startpos, startvel, tstart):

    # Pick a move time: use the time it would take to move the desired
    # distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
    # 1sec min time.  Note this is arbitrary/approximate - we could
    # also compute the fastest possible time or pass as an argument.
    AVGSPEED = 1.5
    MINTIME  = 1.0
    tmove = math.fabs(goalpos - startpos) / AVGSPEED
    if (tmove < MINTIME):
        tmove = MINTIME

    # Set the cubic spline parameters.  Make sure the main code (timer)
    # doesn't access until we're done.
    global t0, tf, a, b, c, d
    with access_parameters_mutex:
        t0 = tstart
        tf = tstart + tmove
        a  = startpos
        b  = startvel
        c  = ( 3.0 * (goalpos - startpos) / tmove + 2.0 * startvel) / tmove
        d  = (-2.0 * (goalpos - startpos) / tmove - 3.0 * startvel) / (tmove*tmove)

    # Report.
    rospy.loginfo("Moving from %6.3frad to %6.3frad over %5.3fsec" %
                  (startpos, goalpos, tmove))


#
#   Goal Subscriber Callback
#
#   This message is of type std_msgs::Float64, i.e. it contains only
#   one number.  Use that as a new goal position, recomputing the
#   cubic spline.
#
def goalCallback(msg):
    # Reset the trajectory (cubic spline) parameters to reach the goal
    # starting at the current values. 
    if(len(msg.faces) >=1):
	x = msg.faces[0].face.x
        print(x)
    	setspline(x/500, cmdpos, cmdvel, t)
    

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pymovetoexplicit')

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
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    # Set up the spline to move to zero (starting at t=1).
    setspline(0.0, msg.position[0], 0.0, 1.0)

    t      = 0.0
    cmdpos = msg.position[0]
    cmdvel = 0.0
    cmdtor = 0.0

    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    #rospy.Subscriber("goal", Float64, goalCallback)
    rospy.Subscriber("/detector/faces", FaceArrayStamped, goalCallback)

    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.  Make sure the spline parameters are
        # not changed in the middle of the calculations!
        with access_parameters_mutex:
            # Extend the spline in front of beginning or behind the end.
	    if   (t <= t0):
                r = 0.0
	    elif (t >= tf):
                r = tf - t0
	    else:
                r = t  - t0

	    cmdpos = a + b*r + c*r*r + d*r*r*r
	    cmdvel = b + 2.0*c*r + 3.0*d*r*r
	    cmdtor = 0.0

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = cmdpos
        command_msg.velocity[0]  = cmdvel
        command_msg.effort[0]    = cmdtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
