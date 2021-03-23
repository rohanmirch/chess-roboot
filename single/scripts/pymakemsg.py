#!/usr/bin/env python
#
#   pypickup.py
#
#
import sys
import rospy
import math
import rosbag
import time

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64, String
from opencv_apps.msg import FaceArrayStamped
from single.msg import Num
from threading import Lock
import os

if __name__ == "__main__":
    rospy.init_node('pymakemsg')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.
    pub = rospy.Publisher('/goal', Num, queue_size=5)
    
    #pub.publish(.7, .00, .1)
    #time.sleep(3)
    #pub.publish(.7, .00, .1)
    #time.sleep(3)


    print("sending real first message")
    pub.publish(.2125,0, .3)
    time.sleep(1) 
    
    print("sending  first message")
    #pub.publish(.2125,0, .15)
    #time.sleep(3) 

    print("sending first message")
    #pub.publish(.2125 + .3048, 0, .15)
    #time.sleep(5)
    
    
    #print("sending secod message")
    #pub.publish(.2, 0, .3)
    #time.sleep(6)
    B4 = '''
    pub.publish(.347, -.219, .25)
    
    print("sending second message")
    pub.publish(.347, -.219, .25)
    time.sleep(4)

    print("sending third message")
    pub.publish(.347, -.219, .15)
    time.sleep(4)

    print("sending 4th message")
    pub.publish(-1, -1, -1)
    time.sleep(4)
    
    print("sending 5th message")
    pub.publish(.347, -.219, .25)
    time.sleep(4)
    
    print("sending 6th message")
    pub.publish(-2, -2, -2)
    time.sleep(4)

    '''
    
    #G7
    print("sending second message")
    pub.publish(.52, .064, .25)
    time.sleep(4)

    print("sending third message")
    pub.publish(.52, .064, .15)
    time.sleep(4)

    print("sending 4th message")
    pub.publish(-1, -1, -1)
    time.sleep(4)
    
    print("sending 5th message")
    pub.publish(.52, .064, .25)
    time.sleep(4)
    
    print("sending 6th message")
    pub.publish(-2, -2, -2)
    #time.sleep(4)
    
    #print("sending eith message")
    #pub.publish(.14, .24, .15)
    #time.sleep(4)'''

    b = '''


    os.system('pwd')
    os.system('rostopic pub /goal single/Num "x: 0.3 \ny: 0.2 \nz: 0.4"')
    time.sleep(3.5)
    os.system("^C")
    
    
    print("sending secod message")
    os.system('rostopic pub /goal single/Num "x: 0.3 \ny: 0.2 \nz: 0.15"')
    time.sleep(2)

    print("sending third message")
    os.system('rostopic pub /goal single/Num "x: -1.0 \ny: -1.0 \nz: -1.0"')
    time.sleep(2)
    
    print("sending fourth message")
    os.system('rostopic pub /goal single/Num "x: 0.2 \ny: .2 \nz: .25"')
    time.sleep(2)
    print("sending fifth message")
    os.system('rostopic pub /goal single/Num "x: 0.3 \ny: 0.1 \nz: .15"')
    time.sleep(2.5)

    print("sending sixth message")
    os.system('rostopic pub /goal single/Num "x: -2.0 \ny: -2.0 \nz: -2.0"')
    time.sleep(2)

    print("sending seventh message")
    os.system('rostopic pub /goal single/Num "x: 0.3 \ny: 0.2 \nz: 0.4"')
    time.sleep(2)
    '''

    

    

    
