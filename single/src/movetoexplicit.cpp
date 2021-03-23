/*
**   movetoexplicit.cpp
**
**   Continually (at 100Hz!) send commands to the robot, providing
**   explicit moves to goal locations - using a cubic spline.
*/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <boost/thread/mutex.hpp>


/*
**   Global Variables.  We use global variables so the callbacks can
**   see the state and pass information to the main (timer) loop.
*/
static volatile double  t0;		// Cubic spline start time
static volatile double  tf;		// Cubic spline final time
static volatile double  a;		// Cubic spline t^0 parameter
static volatile double  b;		// Cubic spline t^1 parameter
static volatile double  c;		// Cubic spline t^2 parameter
static volatile double  d;		// Cubic spline t^3 parameter

static volatile double  t;		// Current time (sec)
static volatile double  cmdpos;		// Current cmd position (rad)
static volatile double  cmdvel;		// Current cmd velocity (rad/sec)
static volatile double  cmdtor;		// Current cmd torque (Nm)

// Use a mutex so that parameters are accessed/read/set atomically. 
static boost::mutex  access_parameters_mutex;


/*
**   Reset the spline parameters.
**
**   Compute and load the spline parameters
*/
void setspline(double goalpos, double startpos, double startvel, double tstart)
{
  double  tmove;        // Total move time

  // Pick a move time: use the time it would take to move the desired
  // distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
  // 1sec min time.  Note this is arbitrary/approximate - we could
  // also compute the fastest possible time or pass as an argument.
#define AVGSPEED    (1.5)
#define MINTIME     (1.0)
  tmove = fabs(goalpos - startpos) / AVGSPEED;
  if (tmove < MINTIME)
    tmove = MINTIME;

  // Set the cubic spline parameters.  Make sure the main code (timer)
  // doesn't access until we're done.
  {
    boost::mutex::scoped_lock lock(access_parameters_mutex);

    t0 = tstart;
    tf = tstart + tmove;
    a  = startpos;
    b  = startvel;
    c  = ( 3.0 * (goalpos - startpos) / tmove + 2.0 * startvel) / tmove;
    d  = (-2.0 * (goalpos - startpos) / tmove - 3.0 * startvel) / (tmove*tmove);
  }

  // Report.
  ROS_INFO("Moving from %6.3frad to %6.3frad over %5.3fsec",
	   startpos, goalpos, tmove);
}


/*
**   Goal Subscriber Callback
**
**   This message is of type std_msgs::Float64, i.e. it contains only
**   one number.  Use that as a new goal position, recomputing the
**   cubic spline.
*/
void goalCallback(const std_msgs::Float64::ConstPtr& msgptr)
{
  // Reset the trajectory (cubic spline) parameters to reach the goal
  // starting at the current values.
  setspline(msgptr->data, cmdpos, cmdvel, t);
}


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "movetoexplicit");
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
  // Set up the spline to move to zero (starting at t=1).
  setspline(0.0, msgptr->position[0], 0.0, 1.0);

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

      // Compute the commands.  Make sure the spline parameters are
      // not changed in the middle of the calculations!
      {
	boost::mutex::scoped_lock lock(access_parameters_mutex);
	double r;

	// Extend the spline in front of beginning or behind the end.
	if      (t <= t0)   r = 0.0;
	else if (t >= tf)   r = tf - t0;
	else                r = t  - t0;

	cmdpos = a + b*r + c*r*r + d*r*r*r;
	cmdvel = b + 2.0*c*r + 3.0*d*r*r;
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
