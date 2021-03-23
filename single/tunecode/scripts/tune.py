#!/usr/bin/env python
#
#   tune.py
#
#   Trajectory generator used for tuning.  This allows:
#     (1) moving to particular (nominal) locations
#     (2) overlaying a triangle wave on a single joint
#     (3) floating one or all joints
#
#   It also reloads the gravity calculations whenever it starts
#   floating (for easy updates to the gravity model).
#
#   Note to enable floating, it embeds the command generation into a
#   callback from the actual joint_state.  Not my favorite as it
#   creates timing dependencies, but it makes the tuning easiest.
#
#   External Interface Services:
#     /tune/moveto             Move to given array of joint positions
#     /tune/floatall           Float all joints.  Also reload gravity!
#     /tune/floatone           Float the pulsing joint.  Also reload gravity!
#     /tune/lock               Lock all joints.
#     /tune/pulse              Enable triangle wave overlay
#     /tune/configure          Set parameters: joint/amplitude/period
#
import sys
import math
import rospy

try:
    import gravity
except:
    print("No gravity module!")

from robotutils      import *
from sensor_msgs.msg import JointState
from std_srvs.srv    import Empty, EmptyResponse
from tunecode.srv    import MoveTo, MoveToResponse
from tunecode.srv    import TuneCfg, TuneCfgResponse


#
#   Robot Definition
#
family = 'Dwarfs'
names  = ['Doc', 'Sleepy', 'Grumpy']

dofs      = len(names)
fullnames = [family+'/'+name for name in names]


#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information.  This could also be embedded
#   in a class, but it's the only thing in this file.
#
# General parameters
joint     = 0                           # Joint number
amplitude = 0.1                         # Positive amplitude
period    = 2.0                         # Positive period

# Current nominal position/velocity (without triange wave overlay)
nompos = [0.0] * dofs
nomvel = [0.0] * dofs

# Floating mode (how many joints to float)
class FloatMode:
    OFF = 0                     # No joints are floating
    ONE = 1                     # Float one joint
    ALL = 2                     # Float all joints
floating = FloatMode.OFF

# Splines, one per dof
tspline = []                    # Start time for splines
spline  = [[]] * dofs           # Individual splines
for i in range(dofs):
    spline[i] = Spline()

# Triangle wave parameters - the current parameters are normally
# updated from the requests at the end of a period.
twave      = []                 # Start time of current period
waving     = False              # Currently overlaying
nextwaving = False              # After period, new state of overlaying

wavejoint  = joint                      # Current joint number
waveperiod = period                     # Current triangle wave period
wavespeed  = 4.0 * amplitude / period   # Current triangle wave velocity

# Publisher and the command message.  Note the final commands are the
# the sum of the above command position and the triangle wave overlay.
cmdpub = []
cmdmsg = JointState()
cmdmsg.name = fullnames
cmdmsg.position = [0.0] * dofs
cmdmsg.velocity = [0.0] * dofs
cmdmsg.effort   = [0.0] * dofs


#
#   Float Adjustments
#
def float_off():
    global floating
    floating = FloatMode.OFF

def float_one():
    global floating
    floating = FloatMode.ONE

def float_all():
    global floating
    floating = FloatMode.ALL

#
#   Spline Adjustments
#
def spline_move(tmove, goalpos):
    # Configure the splines, starting now.
    t = (rospy.Time.now() - tspline).to_sec()
    for i in range(dofs):
        spline[i].set(t, nompos[i], nomvel[i], t+tmove, goalpos[i], 0.0)

def spline_moveto(goalpos):
    # Determine an appropriate move time, enforcing a 0.5s min.
    AVGSPEED  = 0.5
    tmove     = 0.5
    for i in range(dofs):
        tmove = max(tmove, math.fabs(goalpos[i] - nompos[i]) / AVGSPEED)
    spline_move(tmove, goalpos)

def spline_stop():
    spline_move(0.5, nompos)

def spline_lock():
    spline_move(0.01, nompos)

#
#   Triangle Wave Overlay Adjustments
#
def wave_instantoff():
    global waving, nextwaving
    waving     = False
    nextwaving = False

def wave_off():
    global nextwaving
    nextwaving = False

def wave_update():
    # Update the parameters to new values, potentially turning on/off.
    global waving, wavejoint, waveperiod, wavespeed
    waving     = nextwaving
    wavejoint  = joint
    waveperiod = period
    wavespeed  = 4.0 * amplitude / period

def wave_on():
    global nextwaving
    nextwaving = True
    # If not active, start a new period and update to turn on.
    if not waving:
        global twave
        twave = rospy.Time.now()
        wave_update()

def wave_shift():
    # Shift by one period and update the parameters.
    global twave
    twave += rospy.Duration.from_sec(waveperiod)
    wave_update()


#
#   Joint State Callback - Generate next command
#
#     /hebiros/robot/feedback/joint_state
#
def cb_loop(msg):
    # Grab the time.
    tloop = rospy.Time.now()

    # Compute the nominal position/velocity based on the floating.
    if (floating == FloatMode.ALL):
        for i in range(dofs):
            nompos[i] = msg.position[i]
            nomvel[i] = msg.velocity[i]
    elif (floating == FloatMode.ONE):
        for i in range(dofs):
            nomvel[i] = 0.0
        nompos[joint] = msg.position[joint]
        nomvel[joint] = msg.velocity[joint]
    else:
        t = (tloop - tspline).to_sec()
        for i in range(dofs):
            (nompos[i], nomvel[i]) = spline[i].calc(t)

    # Populate the command message, adding gravity torques if available.
    for i in range(dofs):
        cmdmsg.position[i] = nompos[i]
        cmdmsg.velocity[i] = nomvel[i]
    try:
        cmdmsg.effort = gravity.gravity(cmdmsg.position)
        if (len(cmdmsg.effort) != dofs):
            cmdmsg.effort = [0.0] * dofs
    except:
        cmdmsg.effort = [0.0] * dofs

    # Overlay the triangle wave
    if waving:
        t = (tloop - twave).to_sec()
        if   (t <= 0.25 * waveperiod):
            cmdmsg.position[wavejoint] += wavespeed * t
            cmdmsg.velocity[wavejoint] += wavespeed
        elif (t <= 0.75 * waveperiod):
            cmdmsg.position[wavejoint] -= wavespeed * (t - 0.5*waveperiod)
            cmdmsg.velocity[wavejoint] -= wavespeed
        else:
            cmdmsg.position[wavejoint] += wavespeed * (t - waveperiod)
            cmdmsg.velocity[wavejoint] += wavespeed
        # If we have exceeded a period, shift and update.
        if (t >= waveperiod):
            wave_shift()

    # Publish
    cmdmsg.header.stamp = tloop
    cmdpub.publish(cmdmsg)


#
#   External Interface Service Callbacks
#
#     /tune/moveto             Move to given array of joint positions
#     /tune/floatall           Float all joints.  Also reload gravity!
#     /tune/floatone           Float the pulsing joint.  Also reload gravity!
#     /tune/lock               Lock all joints.
#     /tune/pulse              Enable triangle wave overlay
#     /tune/configure          Set parameters: joint/amplitude/period
#
def cb_moveto(req):
    # Check the position data.
    if (len(req.position) != dofs):
        rospy.logerr("MoveTo needs %d positions!" % dofs)
        raise ValueError('Bad number of joints')

    # Set splines to move, turn off floating, leave the overlay.
    spline_moveto(req.position)
    float_off()
    rospy.loginfo("Moving to " + str(req.position))
    return MoveToResponse(True)

def cb_floatall(req):
    # Set splines to lock, turn on floating, disable overlay immediately.
    spline_lock()
    float_all()
    wave_instantoff()
    try:
        reload(gravity)
        rospy.loginfo("Floating all joints (gravity reloaded)")
    except:
        rospy.logwarn("Floating all joints - no gravity model!")
    return EmptyResponse()

def cb_floatone(req):
    # Set splines to lock, turn on single floating, disable overlay immediately.
    spline_lock()
    float_one()
    wave_instantoff()
    try:
        reload(gravity)
        rospy.loginfo("Floating joint %d (gravity reloaded)" % joint)
    except:
        rospy.logwarn("Floating joint %d - no gravity model!" % joint)
    return EmptyResponse()

def cb_lock(req):
    # Set splines to stop, turn off floating, disable overlay after period.
    spline_stop()
    float_off()
    wave_off()
    rospy.loginfo("Locking in place")
    return EmptyResponse()

def cb_pulse(req):
    # Set splines to stop, turn off floating, enable overlay after period.
    spline_stop()
    float_off()
    wave_on()
    rospy.loginfo("Overlaying triangle wave on joint %d" % joint)
    return EmptyResponse()

def cb_configure(req):
    # Check the arguments.
    errstr = ''
    if not (req.joint in range(dofs)):
        errstr = "Requested joint number %d is out of range" % req.joint
    elif not (req.amplitude > 0.0):
        errstr = "Requested amplitude %f is not positive" % req.amplitude
    elif not (req.period > 0.0):
        errstr = "Requested period %f is not positive" % req.period
    if errstr:
        rospy.logerr(errstr)
        raise ValueError(errstr)

    # Push the parameters.
    global joint, amplitude, period
    joint     = req.joint
    amplitude = req.amplitude
    period    = req.period
    rospy.loginfo("New joint %d amp %6.3f period %6.3f" %
                  (joint, amplitude, period))
    return TuneCfgResponse(True)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('tune')

    # Define/check the robot joints.
    hebi_define_robot(family, names)

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial commands.
    cmdpub = rospy.Publisher('/hebiros/robot/command/joint_state',
                             JointState, queue_size=10)
    rospy.sleep(0.4)


    # Grab the initial position/velocity/effort/gravity.
    (initpos, initvel, initeff) = get_joints()
    try:
        initgrav = gravity.gravity(initpos)
    except:
        initgrav = [0.0] * dofs
        rospy.logwarn("No gravity model!")
    if (len(initgrav) != dofs):
        rospy.logerr("Gravity model returned incorrect number of joints!")
        raise ValueError("Bad gravity model number of joints")

    # Start commanding, ramping up the effort.
    N = 50
    for n in range(N):
        cmdmsg.header.stamp = rospy.Time.now()
        for i in range(dofs):
            cmdmsg.position[i] = initpos[i]
            cmdmsg.velocity[i] = 0.0
            cmdmsg.effort[i]   = initeff[i] + n * ((initgrav[i] - initeff[i])/N)
        cmdpub.publish(cmdmsg)
        rospy.sleep(0.01)


    # Initialize the nominal position/velocity and timing.
    for i in range(dofs):
        nompos[i] = initpos[i]
        nomvel[i] = 0.0

    tspline = rospy.Time.now()
    twave   = rospy.Time.now()

    # Start holding where we are (i.e. locking).
    spline_lock()
    float_off()
    wave_off()


    # Now that the variables are all valid, create/enable the
    # subscriber and services which will kick off the operation.
    rospy.Subscriber('/hebiros/robot/feedback/joint_state', JointState, cb_loop)

    rospy.Service('~moveto',     MoveTo,  cb_moveto)
    rospy.Service('~floatall',   Empty,   cb_floatall)
    rospy.Service('~floatone',   Empty,   cb_floatone)
    rospy.Service('~lock',       Empty,   cb_lock)
    rospy.Service('~pulse',      Empty,   cb_pulse)
    rospy.Service('~configure',  TuneCfg, cb_configure)

    # Report and spin (until shut down).
    rospy.loginfo("Tuning is runing...")
    rospy.spin()
