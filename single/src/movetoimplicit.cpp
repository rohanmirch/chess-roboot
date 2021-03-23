/*
**   movetoimplicit.cpp
**
**   Continually (at 100Hz!) send commands to the robot, providing
**   implicit moves to goal locations - using a filter.
*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <boost/thread/mutex.hpp>


/*
**   Global Variables.  We use global variables so the callbacks can
**   see the state and pass information to the main (timer) loop.
*/
static volatile double  goalpos;	// Goal position

static volatile double  t;		// Current time (sec)
static volatile double  cmdpos;		// Current cmd position (rad)
static volatile double  cmdvel;		// Current cmd velocity (rad/sec)
static volatile double  cmdtor;		// Current cmd torque (Nm)

// We don't need a mutex here, as there is a single parameter (which
// hence will always be self-consistent!)
// static boost::mutex  access_parameters_mutex;


/*
**   Goal Subscriber Callback
**
**   This message is of type std_msgs::Float64, i.e. it contains only
**   one number.  Use that as a new goal position.
*/
void goalCallback(const std_msgs::Float64::ConstPtr& msgptr)
{
  // Simply save the new goal position.
  goalpos = msgptr->data;

  // Report.
  ROS_INFO("Moving goal to %6.3frad", goalpos);
}


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "movetoimplicit");
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
  goalpos = 0.0;

  t      = 0.0;
  cmdpos = msgptr->position[0];
  cmdvel = 0.0;
  cmdtor = 0.0;

  // Now that the variables are valid, create/enable the subscriber
  // that (at any time hereafter) may read/update the settings.
  ros::Subscriber freqSubscriber = n.subscribe("goal", 10, goalCallback);

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

      // Adjust the commands, effectively filtering the goal position
      // into the command position.  Note we only use a single
      // parameter (goalpos) which we read just once, so there is no
      // chance of self-inconsistency.  I.e. we don't need to mutex!
#define TIMECONSTANT (0.7)		// Convergence time constant
#define LAMBDA	     (1.0/TIMECONSTANT)	// Convergence rate
#define MAXVELOCITY  (1.5)		// Velocity magnitude limit
      double cmdacc;

      cmdacc = - 1.4 * LAMBDA * cmdvel - LAMBDA*LAMBDA* (cmdpos - goalpos);
      cmdvel = cmdvel + dt * cmdacc;
      if      (cmdvel >  MAXVELOCITY)  cmdvel =  MAXVELOCITY;
      else if (cmdvel < -MAXVELOCITY)  cmdvel = -MAXVELOCITY;
      cmdpos = cmdpos + dt * cmdvel;

      cmdtor = 0.0;

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
