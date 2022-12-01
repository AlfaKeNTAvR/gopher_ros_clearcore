#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gopher_ros_clearcore.srv import *


def serial_write(command):
    # TODO: Call a serial write service
    serial_write_srv(command)


# Topic callback functions
# Stop the motion
def stop_callback(req):
    command = "vm_0.0_"   
    serial_write(command)


# Velocity movement
def vel_callback(req):
    global last_vel
    command = ""
    vel = req.linear.z

    # Send only when velocity value changes
    if abs(last_vel - vel) > 0.01:
        last_vel = vel
        command = "vm_" + str(vel) + "_"   
        serial_write(command)
    

# Service handlers
# Drive
def drive_control_handler(req):
    command = ""

    # Enable
    if req.command == True:
        command = "ed_"

    # Disable
    elif req.command == False:
        command = "dd_"

    serial_write(command)
    
    # Service response
    response = req.command

    return response


# Brake
def brake_control_handler(req):
    command = ""

    # Enable
    if req.command == True:
        command = "eb_"

    # Disable
    elif req.command == False:
        command = "db_"

    serial_write(command)

    # Service response
    response = req.command

    return response


# Debug mode
def debug_control_handler(req):
    command = ""

    # Enable
    if req.command == True:
        command = "debug_on_"

    # Disable
    elif req.command == False:
        command = "debug_off_"
    
    serial_write(command)

    # Service response
    response = req.command

    return response


# Logger mode
def logger_control_handler(req):
    command = ""

    # Enable
    if req.command == True:
        command = "logger_on_1000_"

    # Disable
    elif req.command == False:
        command = "logger_off_"
    
    serial_write(command)

    # Service response
    response = req.command

    return response


# Homing
def homing_handler(req):
    command = ""

    # Start homing
    if req.command == True:
        command = "hm_"

    serial_write(command)
    
    # Service response
    response = req.command

    return response


# Relative position movement
def abspos_handler(req):
    pos = req.position
    vel = req.velocity
    command = "am_" + str(pos) + "_" + str(vel) + "_"     
    serial_write(command)

    # Service response
    response = True

    return response


# Relative position movement
def relpos_handler(req):
    pos = req.position
    vel = req.velocity
    command = "rm_" + str(pos) + "_" + str(vel) + "_"      
    serial_write(command)

    # Service response
    response = True

    return response


if __name__ == "__main__":
    # Variables
    last_vel = 0.0

    # Initialize a node
    rospy.init_node("z_chest_control", anonymous=True)

    # Subscribe to topics
    rospy.Subscriber('z_chest_stop', Bool, stop_callback)
    rospy.Subscriber('z_chest_vel', Twist, vel_callback)

    # Initialize services
    drive_control_srv = rospy.Service('z_chest_drive', DriveControl, drive_control_handler)
    brake_control_srv = rospy.Service('z_chest_brake', BrakeControl, brake_control_handler)
    debug_control_srv = rospy.Service('z_chest_debug', DebugControl, debug_control_handler)
    logger_control_srv = rospy.Service('z_chest_logger', LoggerControl, logger_control_handler)
    homing_srv = rospy.Service('z_chest_home', Homing, homing_handler)
    abspos_srv = rospy.Service('z_chest_abspos', AbsolutePosition, abspos_handler)
    relpos_srv = rospy.Service('z_chest_relpos', RelativePosition, relpos_handler)

    # Add services
    serial_write_srv = rospy.ServiceProxy('serial_write', SerialWrite)

    print("Gopher Chest control is ready.")

    while not rospy.is_shutdown():
        pass

    print("\nShutting down...")

