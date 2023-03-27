#!/usr/bin/env python
# TODO: Rewrite the code as a Class.
"""Controls a robot chest component.

Initializes a ROS node 'z_chest_control' and sets up various ROS subscribers and
service providers for controlling a robot chest component. It receives velocity
and position messages from ROS topics 'z_chest_vel' and 'z_chest_pos',
respectively, and uses the messages to control the chest.

Also provides various ROS services for controlling the chest, including:

'z_chest_stop' to stop the chest. 
'z_chest_drive' to control the chest drive.
'z_chest_brake' to control the chest brake. 
'z_chest_debug' to enable/disable debug mode. 
'z_chest_logger' to enable/disable logger mode. 
'z_chest_home' to start homing. 
'z_chest_abspos' to move the chest to an absolute position.
'z_chest_relpos' to move the chest to a relative position.

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com)
"""

import rospy
import time

from geometry_msgs.msg import Twist

import gopher_ros_clearcore.msg as ros_clearcore_msg
import gopher_ros_clearcore.srv as ros_clearcore_srv


def velocity_callback(msg):
    """Callback function for the 'z_chest_vel' topic.

    Sends the target velocity of the chest component to the ClearCore board over
    serial communication.

    Args:
        msg (geometry_msgs.msg.Twist): The message received on the 'z_chest_vel'
        topic.
    """

    global last_velocity
    serial_command = ''
    # TODO: Shift from using Twist message, to a float32.
    velocity = msg.linear.z

    current_time['velocity'] = time.perf_counter()

    # Send target velocity at 10 Hz rate.
    if (
        current_time['velocity'] - previous_time['velocity'] > 0.1 and
        abs(velocity - last_velocity) > 0.01
    ):
        previous_time['velocity'] = current_time['velocity']
        last_velocity = velocity

        serial_command = f'vm_{velocity}_'
        serial_write_srv(serial_command)


def position_callback(msg):
    """Callback function for the 'z_chest_pos' topic.

    Sends the target position and velocity of the chest component to the
    ClearCore board to control the chest movement.

    Args:
        msg (gopher_ros_clearcore.msg.Position): The message received on the
        'z_chest_pos' topic.
    """

    serial_command = ''

    current_time['position'] = time.perf_counter()

    # Rate 10 Hz
    if current_time['position'] - previous_time['position'] > 0.1:
        previous_time['position'] = current_time['position']

        serial_command = f'am_{msg.position}_{msg.velocity}_'
        serial_write_srv(serial_command)


# Service handlers TODO: Add proper service responses based on messages from
# ClearCore.
def stop_handler(req):
    """Handler for the 'z_chest_stop' service.

    Sends a serial command to the ClearCore board to stop the motion of the
    chest component.

    Args:
        req (gopher_ros_clearcore.srv.Stop): The request received on the
        'z_chest_stop' service.
    """

    serial_command = 'vm_0.0_'  # Send 0 velocity to stop any motion.
    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def drive_control_handler(req):
    """Handler for the 'z_chest_drive' service.

    Sends a serial command to the ClearCore board to enable or disable the drive
    motor of the chest component.

    Args:
        req (gopher_ros_clearcore.srv.DriveControl): The request received on
        'z_chest_drive' service.
    """

    serial_command = ''

    if req.command:
        serial_command = 'ed_'  # Enable

    elif not req.command:
        serial_command = 'dd_'  # Disable

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def brake_control_handler(req):
    """Handler for the 'z_chest_brake' service.
    
    Sends a serial command to the ClearCore board to enable or disable the brake
    of the chest component.

    Args:
        req (gopher_ros_clearcore.srv.BrakeControl): The request received on
        'z_chest_brake' service.
    """

    serial_command = ''

    if req.command:
        serial_command = 'eb_'  # Enable

    elif not req.command:
        serial_command = 'db_'  # Disable

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def debug_control_handler(req):
    """Handler for the 'z_chest_debug' service.

    Sends a serial command to the ClearCore board to enable or disable the debug
    mode of the chest component.

    Args:
        req (gopher_ros_clearcore.srv.DebugControl): The request received on
        'z_chest_debug' service.
    """

    serial_command = ''

    if req.command:
        serial_command = 'debug_on_'

    elif not req.command:
        serial_command = 'debug_off_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def logger_control_handler(req):
    """Handler for the 'z_chest_logger' service.

    Sends a serial command to the ClearCore board to enable or disable the
    logger mode of the chest component.

    Args:
        req (gopher_ros_clearcore.srv.LoggerControl): The request received on
        'z_chest_logger' service.
    """

    serial_command = ''

    if req.command:
        serial_command = 'logger_on_100_'  # Run logger at 10 Hz.

    elif not req.command:
        serial_command = 'logger_off_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def homing_handler(req):
    """Handler for the 'z_chest_home' service.

    Sends a serial command to the ClearCore board to initiate the homing process
    of the chest component.

    Args:
        req (gopher_ros_clearcore.srv.Homing): The request received on
        'z_chest_home' service.
    """

    serial_command = ''

    # Start homing
    if req.command:
        serial_command = 'hm_'

    serial_write_srv(serial_command)

    # Service response
    response = req.command

    return response


def absolute_position_handler(req):
    """Handles the 'z_chest_abspos' service request.

    Sends a serial command to the ClearCore board to move the chest component.
    to an absolute position with the specified speed fraction.

    Args:
        req (gopher_ros_clearcore.srv.AbsolutePosition): The request received on
        'z_chest_abspos' service. Contains the target position and speed
        fraction.
    """

    serial_command = f'am_{req.position}_{req.velocity}_'
    serial_write_srv(serial_command)

    # Service response
    response = True

    return response


def relative_position_handler(req):
    """Handles the 'z_chest_relpos' service request.

    Sends a serial command to the ClearCore board to move the chest component.
    to a position relative to the current position with the specified speed
    fraction.

    Args:
        req (gopher_ros_clearcore.srv.RelativePosition): The request received on
        'z_chest_relpos' service. Contains the relative position and speed
        fraction.
    """

    serial_command = f'rm_{req.position}_{req.velocity}_'
    serial_write_srv(serial_command)

    # Service response
    response = True

    return response


def node_shutdown():
    """Executes cleanup tasks when the node is shutting down."""

    # Deactivate the logger to avoid serial buffer overflow.
    logger_control_srv(False)

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
        position_callback,
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
        absolute_position_handler,
    )
    rospy.Service(
        'z_chest_relpos',
        ros_clearcore_srv.RelativePosition,
        relative_position_handler,
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