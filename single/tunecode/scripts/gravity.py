#
#   gravity.py
#
#   Gravity Torque Computation
#
import math


#
#   Gravity Torque
#
def gravity(pos):
    # Set the torque offsets - these should really be cleared by
    # reseting the actuator force sensors...
    off0 = 0.0
    off1 = 0.0
    off2 = 0.15
    #off3 = 10

    # Compute from the tip inward
    grav2 =   -.2 * math.cos(pos[2]-pos[1])
    grav1 = - 1.7 * math.cos(pos[1]) - grav2
    grav0 = 0.0

    # Return the gravity vector
    return [0.0, grav0+off0, grav1+off1, grav2+off2, -4.5*math.cos(pos[4])]
