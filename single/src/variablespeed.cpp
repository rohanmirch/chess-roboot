/*
**   variablespeed.cpp
**
**   Continually (at 100Hz!) send sinusoidal commands to the robot -
**   but listen for and change the sinusoid frequency.
*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <boost/thread/mutex.hpp>


/*
**   Global Variables.  We use global variables so the callbacks can
**   see the state and pass information to the main (timer) loop.
*/
static volatile double  offset;		// Sinusoid offset (rad)
static volatile double  amplitude;	// Sinusoid amplitude (rad)
static volatile double  omega;		// Sinusoid omega (rad/sec)
static volatile double  phaseshift;	// Sinusoid phase shift (rad)

static volatile double  t;		// Current time (sec)
static volatile double  cmdpos;		// Current cmd position (rad)
static volatile double  cmdvel;		// Current cmd velocity (rad/sec)
static volatile double  cmdtor;		// Current cmd torque (Nm)

// Use a mutex so that parameters are accessed/read/set atomically. 
static boost::mutex  access_parameters_mutex;


/*
**   Frequency Subscriber Callback
**
**   This message is of type std_msgs::Float64, i.e. it contains only
**   one number.  Use that to set the frequency.  But to keep the
**   position smooth, also adjust the phase!
*/
void freqCallback(const std_msgs::Float64::ConstPtr& msgptr)
{
  double  newfrequency = msgptr->data;
  double  newomega     = 2.0*M_PI * newfrequency;

  // Adjust the phaseshift to guaranetee a smooth position transition
  // (yes, the velocity will jump!).  And record the new omega.  Make
  // sure the main code (timer) doesn't access until we're done.
  {
    boost::mutex::scoped_lock lock(access_parameters_mutex);

    phaseshift = phaseshift + (omega - newomega) * t;
    omega      = newomega;
  }

  // Report.
  ROS_INFO("Adjusted to %f rad/sec (%f Hz), phaseshift %f",
	   newomega, newfrequency, phaseshift);
}


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "variablespeed");
  ros::NodeHandle n;

  // Create a publisher to send commands to the robot.  Also allocate
  // space for the message data.
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/robot/command/joint_state", 10);

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("Dwarfs/Doc");	/* Replace Family/Name */
  command_msg.position.resize(1);
  command_msg.velocity.resize(1);
  command_msg.effort.resize(1);

  // Find the starting position and use as an offset for the sinusoid.
  // This will block, but that's appropriate as we don't want to start
  // until we have this information.  Make sure the joints are in the
  // same order in definerobot as here - else things won't line up!
  sensor_msgs::JointState::ConstPtr msgptr;
  msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/hebiros/robot/feedback/joint_state", n);
  if (msgptr == NULL)
    {
      ROS_ERROR("Failed to receive topic /hebiros/robot/feedback/joint_state");
      return 1;
    }

  // Initialize the parameters and state variables.  Do this before
  // the subscriber is activated (as it may run anytime thereafter).
  offset     = msgptr->position[0];
  amplitude  = M_PI/4.0;
  omega      = 2.0*M_PI * 0.25;
  phaseshift = 0.0;

  t      = 0.0;
  cmdpos = offset;
  cmdvel = 0.0;
  cmdtor = 0.0;

  // Now that the variables are valid, create/enable the subscriber
  // that (at any time hereafter) may read/update the settings.
  ros::Subscriber freqSubscriber = n.subscribe("frequency", 10, freqCallback);

  // Create and run a servo loop at 100Hz until shutdown.
  ros::Rate servo(100);
  double    dt = servo.expectedCycleTime().toSec();
  ROS_INFO("Running the servo loop with dt %f", dt);

  ros::Time starttime = ros::Time::now();
  ros::Time servotime;
  while(ros::ok())
    {
      // Current time (since start).
      servotime = ros::Time::now();
      t = (servotime - starttime).toSec();

      // Compute the commands.  Make sure the omega and phaseshift
      // parameters are not changed in the middle of the calculations!
      {
	boost::mutex::scoped_lock lock(access_parameters_mutex);

	cmdpos = offset + amplitude * sin(omega * t + phaseshift);
	cmdvel =          amplitude * cos(omega * t + phaseshift) * omega;
	cmdtor = 0.0;
      }

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
