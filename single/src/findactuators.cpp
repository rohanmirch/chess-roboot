/*
**   findactuators.cpp
**
**   Simply call the hebiros_node to find the list of actuators.
*/
#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "findactuators");
  ros::NodeHandle n;

  // You can choose to wait until the hebiros_node is running (or else
  // fail the below).
  ROS_INFO("Waiting for the hebiros_node...");
  ros::service::waitForService("/hebiros/entry_list", -1);

  // To communicate with the Hebi node, we create a client for their
  // service.  Note the service arguments (to be communicated both to
  // and from the service) are defined by hebiros::EntryListSrv.
  ros::ServiceClient  client = n.serviceClient<hebiros::EntryListSrv>("/hebiros/entry_list");

  // We also need to instantiate the service arguments.  Note the
  // service requires no outgoing data.  Hence we don't need to
  // initialize/set anything.
  hebiros::EntryListSrv  args;

  // Finally call the service.
  ROS_INFO("Calling /hebiros/entry_list...");
  if (client.call(args))
    {
      ROS_INFO("Found %d actuators:", args.response.entry_list.size);
      for (int i = 0 ; i < args.response.entry_list.size ; i++)
	ROS_INFO("#%d: family '%s', name '%s'", i,
		 args.response.entry_list.entries[i].family.c_str(),
		 args.response.entry_list.entries[i].name.c_str());
    }
  else
    {
      ROS_ERROR("Failed to call /hebiros/entry_list");
      return 1;
    }


  // We'll stop here for now.
  return 0;
}
