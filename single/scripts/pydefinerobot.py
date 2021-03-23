#!/usr/bin/env python
#
#   pydefinerobot.py
#
#   Tell the hebiros_node how to define the robot.

import sys
import rospy

from hebiros.srv import AddGroupFromNamesSrv, SizeSrv


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pydefinerobot')

    # You can choose to wait until the hebiros_node is running (or
    # else fail the below).
    rospy.loginfo("Waiting for the hebiros_node...")
    rospy.wait_for_service('/hebiros/add_group_from_names')

    # To communicate with the Hebi node, we create a proxy for their
    # service.  Note the service arguments (to be communicated both to
    # and from the service) are set by hebiros::AddGroupFromNamesSrv.
    proxy = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)

    # Call the service.
    rospy.loginfo("Calling /hebiros/add_group_from_names...")
    try:
        resp = proxy(group_name='robot', names={'5','6','1','2', '3'}, families={'Photon'})
        rospy.loginfo("'robot' created")
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)


    # Let's check the resulting robot's number of actuators...
    rospy.wait_for_service('/hebiros/robot/size')
    size_proxy = rospy.ServiceProxy('/hebiros/robot/size', SizeSrv);
    try:
        resp = size_proxy()
        rospy.loginfo("'robot' has size %d" % resp.size);
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
