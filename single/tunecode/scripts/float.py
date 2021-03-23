#!/usr/bin/env python
#
#   float.py
#
#   Float the robot using only the gravity model.
#
import math
import rospy

from robotutils import *
from gravity    import *


#
#   Robot Definition
#
family = 'Dwarfs'
names  = ['Doc', 'Sleepy', 'Grumpy']

fullnames = [family+'/'+name for name in names]


#
#   Floater Class
#
#   Listen to the actual joint state and publish matching commands.
#
class Floater:
    def __init__(self, pub, cmdmsgm, gravity=None):
        # Store references to the command publisher and messages.
        self.pub     = pub
        self.cmdmsg  = cmdmsg
        self.gravity = gravity

        # Create a subscriber to listen to joint_states.
        rospy.Subscriber('/hebiros/robot/feedback/joint_state',
                         JointState, self.process)

    def process(self, msg):
        # Simply transfer the header, position, and velocity.
        self.cmdmsg.header   = msg.header
        self.cmdmsg.position = msg.position
        self.cmdmsg.velocity = msg.velocity

        # Add gravity efforts if available, else zero.
        if callable(self.gravity):
            self.cmdmsg.effort = self.gravity(self.cmdmsg.position)
        else:
            self.cmdmsg.effort = [0.0] * len(msg.position)

        # And publish.
        self.pub.publish(self.cmdmsg)


#
#   Main Routine
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('float')

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
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)
    pub.publish(cmdmsg)


    # Gradually change the effort to the gravity model.
    blocking_rampeffort(pub, cmdmsg, gravity(cmdmsg.position))

    # Then move from the current (initial) to the nominal.
    # blocking_moveto(pub, cmdmsg, [0.0, -math.pi/4.0, 0.0], gravity)


    # Finally create the "Floater" object and allow it to float the robot.
    rospy.loginfo("Floating the robot...")
    floater = Floater(pub, cmdmsg, gravity)

    # Use a 5Hz servo to print until shutdown.
    servo = rospy.Rate(5)
    while not rospy.is_shutdown():
        show_joints()
        servo.sleep()
