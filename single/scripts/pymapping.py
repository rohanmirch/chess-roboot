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
import numpy as np
import cv2
import detectmarkers as detect

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64, String
from opencv_apps.msg import FaceArrayStamped
from single.msg import Num
from threading import Lock
import pickle
import os

M = np.array([[ 7.56393057e-06,  5.87162978e-04,  5.88591924e-02],
 [-5.83458168e-04, -4.16271360e-06,  5.53008796e-01],
 [ 2.81921638e-05,  2.02543931e-07,  1.00000000e+00]])

def calibrate():
    global M
    # Set (or get from some input) the matching pixel and
    # coordinates.  4 points work.
    coords = detect.get_corners(True, 1000) # Get the coordinates of the points using dtectmarkers.py
                                           # Dear Rohan, idk if this is where this goes, it takes 
                                           # ~5 seconds every time to run this callibration
    uvlist = coords[0][0].astype(float)
    xylist = coords[1][0].astype(float)
    #uvlist = np.float32([[558, 81], [1513, 35], [1513, 990], [574, 988]])
    #xylist = np.float32([[.14, .24], [.127, -.264], [.653, -.264], [.645, .225]])
    print(uvlist)
    print(xylist)
    # Compute the mapping.
    (M, _) = cv2.findHomography(uvlist, xylist)
    print(M)

def apply(uv):
    # Pick some pixel values (or grab from somewhere):
    # uv = np.float32([320, 240])

    # Transform.  Note the code assumes a set of lists of
    # coordinate.  Reshape accordingly.
    xy = cv2.perspectiveTransform(uv.reshape(1,-1,2), M).reshape(2)
    print(xy)

if __name__ == "__main__":
    #rospy.init_node('pymapping')

    calibrate()

    #Dump M object 
    pickle_out = open("mapping_arr.pickle","wb")
    pickle.dump(M, pickle_out)
    pickle_out.close()

    apply(np.array([1573., 894.]))


   

    

    

    

    
