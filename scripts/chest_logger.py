#!/usr/bin/env python
# TODO: Rewrite the code as a Class and add a main() function.
"""Publishes status information about a robot chest component.

This module subscribes to a ROS topic 'logged_info', which publishes
status information of a robot chest component in the form of a JSON
string. The JSON string is deserialized to obtain velocity, position
and status of the components of the chest, which are then published
on various ROS topics as follows:

'chest_position' topic publishes position of the chest.
'chest_velocity' topic publishes velocity of the chest.
'brake_status' topic publishes status of the chest brake.
'motor_status' topic publishes status of the chest motor.
'limits_status' topic publishes status of the chest limit switches.
This module also initializes a ROS node 'chest_logger' and a ROS
service client 'z_chest_logger' for controlling the logger.

Author(s):
    1. Yveder Joseph (ygjoseph@wpi.edu) 
    2. Nikita Boguslavskii (bognik3@gmail.com)
"""

import rospy
import json

from std_msgs.msg import String
from geometry_msgs.msg import (
    Point,
    Twist,
)

from gopher_ros_clearcore.srv import LoggerControl


# Logger callback function
def logger_callback(msg):
    """Callback function for the ROS logged_info topic subscriber.

    This method is called every time a message is received on the 'logged_info' 
    topic. It deserializes the message (which is a JSON string), and publishes 
    the position, velocity, and status information of the chest on various ROS 
    topics.
    """

    if len(msg.data) > 0:
        # Serialze String to Json
        status = json.loads(msg.data)

        # Object for publishig velocity
        velocity = Twist()
        velocity.linear.z = float(status['Motor']['CurrentVelocity'])
        velocity_pub.publish(velocity)

        # Object for publishig position
        position = Point()
        position.z = float(status['Motor']['CurrentPosition'])
        position_pub.publish(position)

        # Object for publishig status of components of chest
        brake_pub.publish(json.dumps(status['Brake']))
        motor_pub.publish(json.dumps(status['Motor']))
        end_stop.publish(json.dumps(status['Limits']))


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('chest_logger', anonymous=True)

    # Topic publisher
    position_pub = rospy.Publisher(
        'chest_position',
        Point,
        queue_size=1,
    )
    velocity_pub = rospy.Publisher(
        'chest_velocity',
        Twist,
        queue_size=1,
    )
    brake_pub = rospy.Publisher(
        'brake_status',
        String,
        queue_size=1,
    )
    motor_pub = rospy.Publisher(
        'motor_status',
        String,
        queue_size=1,
    )
    end_stop = rospy.Publisher(
        'limits_status',
        String,
        queue_size=1,
    )

    # Topic subscriber
    logger_subscriber = rospy.Subscriber(
        'logged_info',
        String,
        logger_callback,
    )

    # Service subscriber
    logger_control_srv = rospy.ServiceProxy('z_chest_logger', LoggerControl)

    print('Logger is ready.')

    while not rospy.is_shutdown():
        pass