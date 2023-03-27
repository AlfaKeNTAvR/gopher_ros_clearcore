#!/usr/bin/env python
# TODO: Rewrite the code as a Class.

import rospy
import time

from geometry_msgs.msg import Twist

import gopher_ros_clearcore.msg as ros_clearcore_msg
import gopher_ros_clearcore.srv as ros_clearcore_srv


# Topic callback functions
def velocity_callback(msg):  # Velocity movement
    global last_velocity
    serial_command = ''
    # TODO: Shift from using Twist message, to a float32.
    velocity = msg.linear.z

    current_time['velocity'] = time.perf_counter()

    # Sent target velocity at 10 Hz rate.
    if (
        current_time['velocity'] - previous_time['velocity'] > 0.1 and
        abs(velocity - last_velocity) > 0.01
    ):
        previous_time['velocity'] = current_time['velocity']
        last_velocity = velocity

        serial_command = f'vm_{velocity}_'
        serial_write_srv(serial_command)


def pos_callback(msg):  # Absolute position movement
    serial_command = ''

    current_time['position'] = time.perf_counter()

    # Rate 10 Hz
    if current_time['position'] - previous_time['position'] > 0.1:
        previous_time['position'] = current_time['position']

        serial_command = f'am_{msg.position}_{msg.velocity}_'
        serial_write_srv(serial_command)


# Service handlers
# TODO: Add proper service responses based on messages from ClearCore.
def stop_handler(req):
    serial_command = 'vm_0.0_'  # Send 0 velocity to stop any motion.
    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def drive_control_handler(req):  # Drive motor control
    serial_command = ''

    if req.command:
        serial_command = 'ed_'  # Enable

    elif not req.command:
        serial_command = 'dd_'  # Disable

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def brake_control_handler(req):  # Brake control
    serial_command = ''

    if req.command:
        serial_command = 'eb_'  # Enable

    elif not req.command:
        serial_command = 'db_'  # Disable

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def debug_control_handler(req):  # Debug mode control
    serial_command = ''

    if req.command:
        serial_command = 'debug_on_'

    elif not req.command:
        serial_command = 'debug_off_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def logger_control_handler(req):  # Logger mode control
    serial_command = ''

    if req.command:
        serial_command = 'logger_on_100_'  # Run logger at 10 Hz.

    elif not req.command:
        serial_command = 'logger_off_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def homing_handler(req):  # Homing
    serial_command = ''

    # Start homing
    if req.command:
        serial_command = 'hm_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def abspos_handler(req):  # Relative position movement
    serial_command = f'am_{req.position}_{req.velocity}_'
    serial_write_srv(serial_command)

    # Service response
    response = True

    return response


def relpos_handler(req):  # Relative position movement
    serial_command = f'rm_{req.position}_{req.velocity}_'
    serial_write_srv(serial_command)

    # Service response
    response = True

    return response


# Is called when the node is shutting down.
def node_shutdown():
    logger_control_srv(False)  # Deactivate the logger.

    print('\nShutting down...')


if __name__ == '__main__':
    # Initialize a node
    rospy.init_node('z_chest_control', anonymous=True)
    rospy.on_shutdown(node_shutdown)

    # Variables
    last_velocity = 0.0
    current_time = {
        'velocity': time.perf_counter(),
        'position': time.perf_counter(),
    }
    previous_time = {
        'velocity': time.perf_counter(),
        'position': time.perf_counter(),
    }

    # Topic subscriber
    rospy.Subscriber(
        'z_chest_vel',
        Twist,
        velocity_callback,
    )
    rospy.Subscriber(
        'z_chest_pos',
        ros_clearcore_msg.Position,
        pos_callback,
    )

    # Service provider
    rospy.Service(
        'z_chest_stop',
        ros_clearcore_srv.Stop,
        stop_handler,
    )
    rospy.Service(
        'z_chest_drive',
        ros_clearcore_srv.DriveControl,
        drive_control_handler,
    )
    rospy.Service(
        'z_chest_brake',
        ros_clearcore_srv.BrakeControl,
        brake_control_handler,
    )
    rospy.Service(
        'z_chest_debug',
        ros_clearcore_srv.DebugControl,
        debug_control_handler,
    )
    rospy.Service(
        'z_chest_logger',
        ros_clearcore_srv.LoggerControl,
        logger_control_handler,
    )
    rospy.Service(
        'z_chest_home',
        ros_clearcore_srv.Homing,
        homing_handler,
    )
    rospy.Service(
        'z_chest_abspos',
        ros_clearcore_srv.AbsolutePosition,
        abspos_handler,
    )
    rospy.Service(
        'z_chest_relpos',
        ros_clearcore_srv.RelativePosition,
        relpos_handler,
    )

    # Service subscriber
    serial_write_srv = rospy.ServiceProxy(
        'serial_write',
        ros_clearcore_srv.SerialWrite,
    )
    logger_control_srv = rospy.ServiceProxy(
        'z_chest_logger',
        ros_clearcore_srv.LoggerControl,
    )

    print('Chest control is ready.')

    while not rospy.is_shutdown():
        pass