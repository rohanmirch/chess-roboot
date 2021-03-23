/*
**   definerobot.cpp
**
**   Tell the hebiros_node how to define the robot.
*/
#include "ros/ros.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "definerobot");
  ros::NodeHandle n;

  // You can choose to wait until the hebiros_node is running (or else
  // fail the below).
  ROS_INFO("Waiting for the hebiros_node...");
  ros::service::waitForService("/hebiros/add_group_from_names", -1);

  // To communicate with the Hebi node, we create a client for their
  // service.  Note the service arguments (to be communicated both to
  // and from the service) are set by hebiros::AddGroupFromNamesSrv.
  ros::ServiceClient  client = n.serviceClient<hebiros::AddGroupFromNamesSrv>("/hebiros/add_group_from_names");

  // We also need to instantiate the service arguments.  This time we
  // need to set the outgoing data.
  hebiros::AddGroupFromNamesSrv  args;
  args.request.group_name = "robot";
  args.request.families = {"Dwarfs"};	/* Set to your family! */
  args.request.names    = {"Doc"};	/* Set to your names! */

  // Finally call the service.
  ROS_INFO("Calling /hebiros/add_group_from_names...");
  if (client.call(args))
    {
      ROS_INFO("'robot' created");
    }
  else
    {
      ROS_ERROR("Failed to call /hebiros/add_group_from_names");
      return 1;
    }


  // Let's check the resulting robot's number of actuators...
  ros::ServiceClient size_client = n.serviceClient<hebiros::SizeSrv>("/hebiros/robot/size");
  hebiros::SizeSrv   size_args;
  size_client.waitForExistence(ros::Duration(-1));
  if (size_client.call(size_args))
    {
      ROS_INFO("'robot' has size %d", size_args.response.size);
    }
  else
    {
      ROS_ERROR("Failed to call /hebiros/robot/size");
      return 1;
    }

  return 0;
}
