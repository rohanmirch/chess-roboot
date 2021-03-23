/*
**   sendcommands.cpp
**
**   Continually (at 100Hz!) send commands to the robot
**   (in hebiros_node).
*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "sendcommands");
  ros::NodeHandle n;

  // Create a publisher to send commands to the robot.  Also allocate
  // space for the message data.
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/robot/command/joint_state", 100);

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("Dwarfs/Doc");	/* Replace Family/Name */
  command_msg.position.resize(1);
  command_msg.velocity.resize(1);
  command_msg.effort.resize(1);


  // Create a servo loop at 100Hz.
  ros::Rate servo(100);

  // Prep the servo loop variables.
  double  dt = servo.expectedCycleTime().toSec();
  double  t;
  double  cmdpos = 0.0;
  double  cmdvel = 0.0;
  double  cmdtor = 0.0;


  // Run the servo loop until shutdown.
  ROS_INFO("Running the servo loop with dt %f", dt);

  ros::Time starttime = ros::Time::now();
  ros::Time servotime;
  while(ros::ok())
    {
      // Current time (since start).
      servotime = ros::Time::now();
      t = (servotime - starttime).toSec();

      // Compute the commands.
      cmdpos = M_PI/4.0 * sin(2.0*M_PI * 0.25 * t);
      cmdvel = M_PI/4.0 * cos(2.0*M_PI * 0.25 * t) * 2.0*M_PI * 0.25;
      cmdtor = 0.0;

      // Ugly way to debug...
      // ROS_INFO("Command position [%f]", cmdpos);

      // Build and send (publish) the command message.
      command_msg.header.stamp = servotime;
      command_msg.position[0]  = cmdpos;
      command_msg.velocity[0]  = cmdvel;
      command_msg.effort[0]    = cmdtor;
      command_publisher.publish(command_msg);

      // Wait for next turn.
      ros::spinOnce();
      servo.sleep();
    }

  return 0;
}
