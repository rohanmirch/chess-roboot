#!/usr/bin/env python
#
#   pyfindactuators.py
#
#   Simply call the hebiros_node to find the list of actuators.

import sys
import rospy

from hebiros.srv import EntryListSrv


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pyfindactuators')

    # You can choose to wait until the hebiros_node is running (or
    # else fail the below).
    rospy.loginfo("Waiting for the hebiros_node...")
    rospy.wait_for_service('/hebiros/entry_list')

    # To communicate with the Hebi node, we create a proxy for their
    # service.  Note the service arguments (to be communicated both to
    # and from the service) are defined by hebiros::EntryListSrv.
    proxy = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
 
    # Call the service.
    rospy.loginfo("Calling /hebiros/entry_list...")
    try:
        resp = proxy()

        rospy.loginfo("Found %d actuators:" % resp.entry_list.size)
        for i in range(0, resp.entry_list.size):
            rospy.loginfo("#%d: family '%s', name '%s'" %
                          (i,
                           resp.entry_list.entries[i].family,
                           resp.entry_list.entries[i].name))

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
