#!/usr/bin/env python
#
#   pymovetoimplicit.py
#
#   Continually (at 100Hz!) send commands to the robot, providing
#   implicit moves to goal locations - using a filter.
#
import sys
import rospy
import math
import rosbag

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64, String
from opencv_apps.msg import FaceArrayStamped
from single.msg import Num
from threading import Lock
import numpy as np

#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information to the main (timer) loop.
#
goalpos = [0.0, 0.0, 0.0, 0.0, 0.0]		    # Goal position
zeropos = [-0.38, -0.37, 0.44, 0.39, -1.25]
                                                
t      = 0.0                        # Current time (sec)
cmdpos = [0.0, 0.0, 0.0, 0.0, 0.0]            # Current cmd position (rad)
cmdvel = [0.0, 0.0, 0.0, 0.0, 0.0]            # Current cmd velocity (rad
cmdtor = [0.0, 0.0, 0.0, 0.0, 0.0]            # Current cmd torque (Nm)

# We don't need a mutex here, as there is a single parameter (which
# hence will always be self-consistent!)
# access_parameters_mutex = Lock()

l1 = .4
l2 = .3

def get_q2(l1, l2, x, z):
    return -1*math.acos((x**2 + z**2 - l1**2 - l2**2)/(2*l1*l2))

def get_q1(l1, l2, x, z):
    q2 = get_q2(l1, l2, x, z)
    q1 = math.atan2(z,x) - math.atan2(l2*math.sin(q2),(l1 + l2*math.cos(q2)))
    
    return q1

def ikin(x, z):
    if x < 0 or z < 0:
        return [0.0, 0.0]
    # Check if outsize of max reach
    max_reach = l1 + l2
    if math.sqrt(x**2 + z**2) > max_reach:
        theta = math.atan(z/x)
        q2 = 0#math.pi
        q1 = theta
        return [q1, q2, 0.0]
    #angles from desmos sim
    q1 = get_q1(l1, l2, x, z)
    q2 = get_q2(l1, l2, x, z) + math.pi

    #have angles from verticle (our frame)
    q1 = math.pi/2 - q1
    q2 = math.pi - q2
    q3 = (q1 + q2) - math.pi/2
    
    #Now change angles for our robot
    return [q1, -q2, -q3]

def ikin2(x, z):
    if x < 0 or z < 0:
        return [0.0, 0.0]
    # Check if outsize of max reach
    max_reach = l1 + l2
    if math.sqrt(x**2 + z**2) > max_reach:
        theta = math.atan2(z,x)
        q2 = 0#math.pi
        q1 = theta
        return [q1, q2, 0.0]
    #angles from desmos sim
    q1 = get_q1(l1, l2, x, z)
    q2 = get_q2(l1, l2, x, z) + math.pi

    #have angles from verticle (our frame)
    q1 = math.pi/2 - q1
    q2 = math.pi - q2
    q3 = (q1 + q2) - math.pi/2
    
    #Now change angles for our robot
    return [q1, -q2, -q3]


#
#   Goal Subscriber Callback
#
#   This message is of type std_msgs::Float64, i.e. it contains only
#   one number.  Use that as a new goal position.
#
def goalCallback(msg):
    # Simply save the new goal position.
    global goalpos
    
    x = msg.x
    y = msg.y
    z = msg.z
    
    if z < .05:
        return
    arr = np.array([x,y,z])
    if np.linalg.norm(arr) > .699:
        
        arr = .699*(arr/np.linalg.norm(arr))
        x = arr[0]
        y = arr[1]
        z = arr[2]
    
    print(x, y, z, 'peenis')

    if x == 0 and y == 0 and z == 0:
        goalpos = zeropos[:]
    
    if x == -1.0 and y == -1.0 and z == -1.0:
        #### Gripper mechanic close
        goalpos[4] = zeropos[4] -.45
    elif x == -2.0 and y == -2.0 and z == -2.0:
        #### Gripper mechanic open
        goalpos[4] = zeropos[4]
    else:
        # base angle
        t1 = -math.atan2(y,x)
        # change x distance to account for rotation angle
        print(math.fabs(t1))
        print(x)
        x = x/math.cos(math.fabs(t1))   
        print(x)
        [t2, t3, t4] = ikin2(x, z)

        goalpos[0] = t1	+ zeropos[0]
        goalpos[1] = t2 + zeropos[1]
        
        goalpos[2] = t3 + zeropos[2]
        goalpos[3] = t4 + zeropos[3]
        goalpos[4] = goalpos[4]
        print(goalpos)
    # Report.
    #rospy.loginfo("motor 0 (#5) Moving goal to %6.3frad" % goalpos[0])
    #rospy.loginfo("motor 1 (#6) moving to %6.3frad" % goalpos[1])
    #rospy.loginfo("motor 2 (#1) moving to %6.3frad" % goalpos[2])


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pymovetoimplicit')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)

    command_msg = JointState()
    command_msg.name.append('Photon/5')
    command_msg.name.append('Photon/6')
    command_msg.name.append('Photon/1')
    command_msg.name.append('Photon/2')
    command_msg.name.append('Photon/3')

   
    #command_msg.position.append(0.0)
    #command_msg.position.append(0.0)
    #command_msg.position.append(0.0)
    command_msg.position.append(zeropos[0])
    command_msg.position.append(zeropos[1])
    command_msg.position.append(zeropos[2])
    command_msg.position.append(zeropos[3])
    command_msg.position.append(zeropos[4])

    command_msg.velocity.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.velocity.append(0.0)


    command_msg.effort.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.effort.append(0.0)

  

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    goalpos = zeropos[:]
    t      = 0.0
    #MSG coming with order 13256, but we send in order 56123
    cmdpos = [msg.position[3],msg.position[4],msg.position[0],msg.position[2], msg.position[1]]
    cmdvel = [0.0, 0.0, 0.0,0.0,0.0]
    cmdtor = [0.0, 0.0, 0.0,0.0,0.0]

    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    rospy.Subscriber("goal", Num, goalCallback)
    #rospy.Subscriber("/detector/faces", FaceArrayStamped, goalCallback)

    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(50)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)
    #f = open('lol.txt', 'w')

    starttime = rospy.Time.now()	
    while not rospy.is_shutdown():
	msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);
	currpos = [msg.position[3],msg.position[4],msg.position[0],msg.position[2], msg.position[1]]
        
        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()
        #f.write("{} {} {}\n".format(t, msg.position[0], msg.position[1]))

        # Adjust the commands, effectively filtering the goal position
        # into the command position.  Note we only use a single
        # parameter (goalpos) which we read just once, so there is no
        # chance of self-inconsistency. 
        #TIMECONSTANT = 0.7	         	# Convergence time constant
        TIMECONSTANT = .7 ## CHAD ADDITION
        LAMBDA       = 2.0/TIMECONSTANT # Convergence rate
        MAXVELOCITY  = 5.0              # Velocity magnitude limit
        for i in range(5):
    	    cmdacc = - 3.0 * LAMBDA * cmdvel[i] - LAMBDA*LAMBDA* (cmdpos[i] - goalpos[i])
    	    cmdvel[i] = cmdvel[i] + dt * cmdacc
    	    if   (cmdvel[i] >  MAXVELOCITY):
    	        cmdvel[i] =  MAXVELOCITY
    	    elif (cmdvel[i] < -MAXVELOCITY):
    	        cmdvel[i] = -MAXVELOCITY
    	    cmdpos[i] = cmdpos[i] + dt * cmdvel[i]

        #cmdtor = [0.0, 0.0, 0.0, 0.0]
        theta3 = currpos[3] - zeropos[3];
        theta1 = currpos[1] - zeropos[1];
        theta2 = currpos[2] - zeropos[2];
        #print(theta3)
        
        t2 = -2.4*math.sin(math.pi-(theta2 - theta1))
        t1 = -6*math.sin(theta1) - t2
        cmdtor = [0, t1, t2, 0.0,0.0]

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position  = cmdpos
        command_msg.velocity  = cmdvel
        command_msg.effort    = cmdtor
        #print(command_msg.position)
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
