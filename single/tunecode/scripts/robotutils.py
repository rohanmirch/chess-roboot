#
#   robotutils.py
#
#   This provides the following robot and HEBI related utilities:
#
#   *) General utils
#
#        visible = ping(ipaddr)
#
#   *) HEBI (node) administration utils
#
#        hebi_assert_node()                     Make sure the node is running
#        hebi_show_actuators()                  Show list of known actuators
#        hebi_define_robot(family, names)       Create the robot
#        hebi_set_gains(fullname, Kp, Kv, lam)  Set an actuator's gains (temp)
#        hebi_set_forcegains(fullname, Kp, Kd, lam)     Set force gains (careful!)
#
#   *) Joint utils
#
#        show_joints()                    Show the current (actual) joint values
#        (pos, vel, eff) = get_joints()   Grab the current (actual) joint values
#
#   *) Spline/Ramp classes
#
#      These set/calculate the splines atomically.
#
#        Spline()
#        Spline().set(t0, p0, v0, tf, pf, vf)
#        (p, v) = Spline().calc(t)
#
#        Ramp()
#        Ramp().set(t0, y0, tf, yf)
#        (y) = Ramp().calc(t)
#
#   *) BLOCKING command routines
#
#      These generate a command sequence over time, i.e. block until
#      the action is complete.
#
#        blocking_rampeffort(pub, cmdmsg, goaleff)
#        blocking_moveto(pub, cmdmsg, goalpos, gravity)
#        blocking_trianglewave(pub, cmdmsg, joint, amplitude, period, gravity)
#
import os
import sys
import math
import rospy

from threading       import Lock

from hebiros.srv     import EntryListSrv
from hebiros.srv     import AddGroupFromNamesSrv
from hebiros.srv     import SendCommandWithAcknowledgementSrv
from hebiros.msg     import CommandMsg
from sensor_msgs.msg import JointState


######################################################################
#
#   General Operating-System Utils
#
######################################################################

#
#   Ping - check a particular IP address ios visible via ping.
#
#
def ping(ipaddr):
    sys.stdout.write("Address " + ipaddr + " is ... ")
    sys.stdout.flush()
    visible = not os.system("ping -c 1 " + ipaddr + " > /dev/null")
    if visible:
        print("visible")
    else:
        print("not reachable")
    return visible


######################################################################
#
#   HEBI Administrative Utils
#
######################################################################

#
#   Confirm (assert) that the HEBI node is running
#
def hebi_assert_node():
    try:
        rospy.wait_for_service('/hebiros/entry_list', 0.5)
    except rospy.ROSException, e:
        rospy.logerr("HEBI ROS NODE NOT RUNNING!")
        raise

#
#   Check and show the HEBI actuator entry list
#
def hebi_show_actuators():
    # Report.
    rospy.loginfo("Checking HEBI actuator list...")

    # Call a proxy to the /hebiros/entry_list service.  The
    # hebiros.srv.EntryListSrv arguments only contain response values.
    try:
        proxy = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
        resp  = proxy()
        rospy.loginfo("Found %d actuators:" % resp.entry_list.size)
        for i in range(0, resp.entry_list.size):
            rospy.loginfo("#%d: family '%s', name '%s'" %
                          (i,
                           resp.entry_list.entries[i].family,
                           resp.entry_list.entries[i].name))
    except rospy.ServiceException, e:
        rospy.logerr("Unable to get actuator list!")
        rospy.logerr("Service call failed: %s"%e)

#
#   Define and confirm the HEBI's definition of robot joints
#
def hebi_define_robot(family, names):
    # Report.
    rospy.loginfo("Defining robot with...")
    for i,name in enumerate(names):
        rospy.loginfo("Joint %d = %s/%s" % (i, family, name))

    # Use the joint_state topic to check existence and joint order.
    jstopic = '/hebiros/robot/feedback/joint_state'

    # Check whether the robot exists and create if not.
    if jstopic in [topic[0] for topic in rospy.get_published_topics('')]:
        # Simple warn (checking order below).
        rospy.logwarn('Robot already exists!')
    else:
        # Call a proxy to the /hebiros/add_group_from_names service.
        # The hebiros.srv.AddGroupFromNamesSrv defines no response.
        try:
            proxy = rospy.ServiceProxy('/hebiros/add_group_from_names',
                                       AddGroupFromNamesSrv)
            resp  = proxy(group_name='robot', names=names, families=[family])
        except rospy.ServiceException, e:
            rospy.logerr("Failed to define the robot!")
            rospy.logerr("Service call failed: %s"%e)
            raise

    # Check the robot joint order!
    msg = rospy.wait_for_message(jstopic, JointState)
    if msg.name != [family+'/'+name for name in names]:
        # Fail - this will cause problems!
        rospy.logerr('ROBOT HAS DIFFERENT ACTUATOR ORDER!!!')
        for i,name in enumerate(msg.name):
            rospy.logerr("Joint %d = %s" % (i, name))
        raise RuntimeError('Incorrect robot actuator order')

#
#   Temporarily (non-persistent) set the gains on one actuator
#
#   Note the units are: Kp (Nm/rad), Kv (PWM/(rad/s)), Lambda (rad/s)
#
def hebi_set_gains(fullname, Kp, Kv, lam):
    # Check that the parameters are non-negative!
    if (Kp < 0.0) or (Kv < 0.0) or (lam < 0.0):
        rospy.logerr('Kp/Kv/Lambda must be non-negative!')
        raise ValueError('Negative gain/frequency values')

    # Convert the filter bandwidth into the filter constant, saturate
    # to 1.0, and revert back to frequency (in rad/s).  Use the Tustin
    # approximation (blinear mapping) for best correspondence.
    lp  = min(1.0, 2.0 * lam / (2000.0 + lam))
    lam = 2000.0 * lp / (2.0 - lp)

    # Report.
    fmt = ("Joint %-20s set to Kp %6.1f Nm/rad, " +
           "Kv %6.2f PWM/(rad/s), Lam %6.1f rad/s = %6.1f Hz")
    rospy.loginfo(fmt % (fullname, Kp, Kv, lam, lam/(2.0*math.pi)))

    # Populate the command with the things we want to change.
    command = CommandMsg()
    command.name = [fullname]
    command.settings.name = [fullname]
    command.settings.save_current_settings = [False]
    command.settings.position_gains.name = [fullname]
    command.settings.position_gains.kp = [Kp]
    command.settings.velocity_gains.name = [fullname]
    command.settings.velocity_gains.kp = [Kv]
    command.settings.velocity_gains.output_lowpass = [lp]
   
    # Call the /hebiros/robot/send_command_with_acknowledgement service.
    service = '/hebiros/robot/send_command_with_acknowledgement'
    try:
        proxy = rospy.ServiceProxy(service, SendCommandWithAcknowledgementSrv)
        resp  = proxy(command)
    except rospy.ServiceException, e:
        rospy.logerr("Failed to set gains!")
        rospy.logerr("Service call failed: %s"%e)
        raise

def hebi_set_forcegains(fullname, Kp, Kd, lam):
    # Check that the parameters are non-negative!
    if (Kp < 0.0) or (Kd < 0.0) or (lam < 0.0):
        rospy.logerr('Kp/Kd/Lambda must be non-negative!')
        raise ValueError('Negative gain/frequency values')

    # Convert the filter bandwidth into the filter constant, saturate
    # to 1.0, and revert back to frequency (in rad/s).  Use the Tustin
    # approximation (blinear mapping) for best correspondence.
    lp  = min(1.0, 2.0 * lam / (2000.0 + lam))
    lam = 2000.0 * lp / (2.0 - lp)

    # Report.
    fmt = ("Joint %-20s FORCE  Kp %6.2f Nm/rad, " +
           "Kd %6.4f PWM/(rad/s), Lam %6.1f rad/s = %6.1f Hz")
    rospy.loginfo(fmt % (fullname, Kp, Kd, lam, lam/(2.0*math.pi)))

    # Populate the command with the things we want to change.
    command = CommandMsg()
    command.name = [fullname]
    command.settings.name = [fullname]
    command.settings.save_current_settings = [False]
    command.settings.effort_gains.name = [fullname]
    command.settings.effort_gains.kp = [Kp]
    command.settings.effort_gains.kd = [Kd]
    command.settings.effort_gains.output_lowpass = [lp]
   
    # Call the /hebiros/robot/send_command_with_acknowledgement service.
    service = '/hebiros/robot/send_command_with_acknowledgement'
    try:
        proxy = rospy.ServiceProxy(service, SendCommandWithAcknowledgementSrv)
        resp  = proxy(command)
    except rospy.ServiceException, e:
        rospy.logerr("Failed to set gains!")
        rospy.logerr("Service call failed: %s"%e)
        raise


######################################################################
#
#   Joint Utils
#
######################################################################

#
#   Show the current joint values
#
def show_joints():
    # Grab a joint_state message.
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)

    # Report.
    rospy.loginfo("Current joint values are:")
    fmt = "Joint %d = %-20s at %6.3frad, vel %6.3frad/sec, with %6.3fNm"
    for i in range(len(msg.name)):
        rospy.loginfo(fmt % (i, msg.name[i], msg.position[i],
                             msg.velocity[i], msg.effort[i]))

#
#   Get the current joint values (as non-empty lists)
#
def get_joints():
    # Grab a joint_state message.
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)

    # Create lists (so that we can mutate later).
    pos = [p for p in msg.position]
    vel = [v for v in msg.velocity]
    eff = [e for e in msg.effort]

    # Make sure the velocity and effort are not empty (the "fake HEBI
    # node" = joint_state_publisher skips these).
    if not vel:
        vel = [0.0] * len(msg.name)
    if not eff:
        eff = [0.0] * len(msg.name)

    # Return the values.
    return (pos, vel, eff)


######################################################################
#
#   Spline/Ramp Classes
#
######################################################################

#
#   Cubic Spline Class
#
class Spline:
    def __init__(self):
        # Use a mutex so that parameters are read/set atomically.
        self.mutex = Lock()

        # Clear the variables.
        self.t0 = 0.0		# Cubic spline start time
        self.T  = 0.01		# Cubic spline move time (relative)
        self.a  = 0.0		# Cubic spline (t-t0)^0 parameter
        self.b  = 0.0		# Cubic spline (t-t0)^1 parameter
        self.c  = 0.0		# Cubic spline (t-t0)^2 parameter
        self.d  = 0.0		# Cubic spline (t-t0)^3 parameter

    def set(self, t0, p0, v0, tf, pf, vf):
        # Make sure we have a positive move time.
        T = max(tf-t0, 0.01)

        # Set the cubic spline parameters (atomically).
        with self.mutex:
            self.t0 = t0
            self.T  = T
            self.a  = p0
            self.b  = v0
            self.c  = ( 3.0*(pf-p0)/T - vf - 2.0*v0) / T
            self.d  = (-2.0*(pf-p0)/T + vf +     v0) / (T*T)

    def calc(self, t):
        # Compute the commands (atomically).
        with self.mutex:
            # Ignore times before and after the spline.
            r = t - self.t0
            if (r < 0.0):
                r = 0.0
	    elif (r > self.T):
                r = self.T

            # Compute the position and velocity.
            p = self.a + r*(self.b + r*(    self.c + r*    self.d))
            v =             self.b + r*(2.0*self.c + r*3.0*self.d)
        return (p,v)

#
#   Ramp Class
#
class Ramp:
    def __init__(self):
        # Use a mutex so that parameters are read/set atomically.
        self.mutex = Lock()

        # Clear the variables.
        self.t0   = 0.0		# Ramp start time
        self.tf   = 0.01	# Ramp final time
        self.y0   = 0.0		# Ramp initial value
        self.yf   = 0.0		# Ramp final value
        self.dydt = 0.0         # Ramp slope

    def set(self, t0, y0, tf, yf):
        # Make sure we have a positive change time.
        T = max(tf-t0, 0.01)

        # Set the ramp parameters (atomically).
        with self.mutex:
            self.t0 = t0
            self.tf = t0 + T
            self.y0 = y0
            self.yf = yf
            self.dydt = (yf - y0) / T

    def calc(self, t):
        # Compute the value (atomically).
        with self.mutex:
            # Ignore times before and after the ramp.
            if   (t < self.t0):
                y = self.y0
            elif (t > self.tf):
                y = self.yf
            else:
                y = self.y0 + self.dydt * (t - self.t0)
        return y


######################################################################
#
#   BLOCKING Change-Effort (by Ramp) and Move-To-Position (by Spline)
#
######################################################################

#
#   Change Effort
#
#   This uses linear ramp to smoothly change effort, from the current
#   command effort to the goal effort.  Leave the pos/vel alone.
#
def blocking_rampeffort(pub, cmdmsg, goaleff):
    # Report
    rospy.loginfo('Adjusting the effort to ' + str(goaleff) + ' ...')

    # Check the number of dofs
    dofs = len(cmdmsg.name)

    # Determine the change time.
    tchange = 0.5

    # Initialize the ramps.
    ramp = [[]] * dofs
    for i in range(dofs):
        ramp[i] = Ramp()
        ramp[i].set(0.0, cmdmsg.effort[i], tchange, goaleff[i])

    # Use a 100Hz servo to send the commands.
    servo     = rospy.Rate(100)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start of move)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Build and publish the command message.
        cmdmsg.header.stamp = servotime
        for i in range(dofs):
            cmdmsg.effort[i] = ramp[i].calc(t)
        pub.publish(cmdmsg)

        # Break/return if the move has completed.  Else wait for the next turn.
        if (t > tchange):
            return
        servo.sleep()

#
#   Move to Goal Position
#
#   This uses cubic splines to smoothly change positions, from the
#   current command position to the goal position.  Use a gravity
#   model if available.
#
def blocking_moveto(pub, cmdmsg, goalpos, gravity=None):
    # Report
    rospy.loginfo('Moving to the goal position ' + str(goalpos) + ' ...')

    # Check the number of dofs
    dofs = len(cmdmsg.name)

    # Determine an appropriate move time, enforcing a 0.5s min.
    AVGSPEED  = 0.5
    tmove     = 0.5
    for i in range(dofs):
        tmove = max(tmove, math.fabs(goalpos[i]-cmdmsg.position[i]) / AVGSPEED)

    # Initialize the splines.
    spline = [[]] * dofs
    for i in range(dofs):
        spline[i] = Spline()
        spline[i].set(0.0, cmdmsg.position[i], cmdmsg.velocity[i],
                      tmove, goalpos[i], 0.0)

    # Use a 100Hz servo to send the commands.
    servo     = rospy.Rate(100)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start of move)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Build and publish the command message.  If we have a gravity
        # model, compute the gravity effort.  Else leave efforts alone.
        cmdmsg.header.stamp = servotime
        for i in range(dofs):
            (cmdmsg.position[i], cmdmsg.velocity[i]) = spline[i].calc(t)
        if callable(gravity):
            cmdmsg.effort = gravity(cmdmsg.position);
        pub.publish(cmdmsg)

        # Break/return if the move has completed.  Else wait for the next turn.
        if (t > tmove):
            return
        servo.sleep()

#
#   Triangle Wave
#
#   Move a single joint according to a triangle wave of given
#   amplitude and period.  Use a gravity model if available.
#
def blocking_trianglewave(pub, cmdmsg, joint, amplitude, period, gravity=None):
    # Report
    rospy.loginfo('Running triangle wave on jnt %d, amp %f, period %f ...' %
                  (joint, amplitude, period))

    # Make sure we have a positive period.
    period = max(period, 0.01)

    # Remember the center and pre-compute the speed.
    center = cmdmsg.position[joint]
    speed  = 4.0 * amplitude / period

    # Use a 100Hz servo to send the commands.
    servo     = rospy.Rate(100)
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start of the last cycle)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Build and publish the command message.  If we have a gravity
        # model, compute the gravity effort.  Leave other values alone.
        cmdmsg.header.stamp = servotime
        if   (t <= 0.25 * period):
            cmdmsg.position[joint] = center + speed * t
            cmdmsg.velocity[joint] =          speed
        elif (t <= 0.75 * period):
            cmdmsg.position[joint] = center - speed * (t - 0.5*period)
            cmdmsg.velocity[joint] =        - speed
        else:
            cmdmsg.position[joint] = center + speed * (t - period)
            cmdmsg.velocity[joint] =        + speed
        if callable(gravity):
            cmdmsg.effort = gravity(cmdmsg.position);
        pub.publish(cmdmsg)

        # Shift the (period's) start time after a full period
        if (t >= period):
            starttime = starttime + rospy.Duration.from_sec(period)

        # Wait for the next turn.
        servo.sleep()
