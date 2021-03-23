/*
**   whereisrobot.cpp
**
**   Find the position of the robot's joints.
*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "whereisrobot");
  ros::NodeHandle n;

  // Grab a single message form the feedback/joint_state topic.
  ROS_INFO("Waiting for topic /hebiros/robot/feedback/joint_state...");
  sensor_msgs::JointState::ConstPtr msgptr;
  msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/hebiros/robot/feedback/joint_state", n);
  if (msgptr == NULL)
    {
      ROS_ERROR("Failed to receive topic /hebiros/robot/feedback/joint_state");
      return 1;
    }

  // Report service.
  ROS_INFO("The robot is at:");
  for (int i = 0 ; i < msgptr->position.size() ; i++)
    ROS_INFO("Joint #%d (name '%s'): position %f", i,
	     msgptr->name[i].c_str(),
	     msgptr->position[i]);
  
  // We'll stop here for now.
  return 0;
}
