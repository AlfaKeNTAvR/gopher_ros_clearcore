#!/usr/bin/env python
# TODO: Rewrite the code as a Class.

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