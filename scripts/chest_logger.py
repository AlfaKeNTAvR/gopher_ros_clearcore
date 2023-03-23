#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from gopher_ros_clearcore.srv import *
from gopher_ros_clearcore.msg import *
import geometry_msgs.msg 
import json

# Logger callback function
def logger_callback(msg):

    data = msg.data

    if len(data) > 0:
        # Serialze String to Json
        status = json.loads(data)

        # Object for publishig velocity
        velocity = geometry_msgs.msg.Twist()
        velocity.linear.z = float(status["Motor"]["CurrentVelocity"])
        velocity_pub.publish(velocity)

        # Object for publishig position
        position =  geometry_msgs.msg.Point()
        position.z = float(status["Motor"]["CurrentPosition"])
        position_pub.publish(position)

        # Object for publishig status of components of chest
        brake_pub.publish(json.dumps(status["Brake"]))
        motor_pub.publish(json.dumps(status["Motor"]))
        end_stop.publish(json.dumps(status["Limits"]))


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('chest_logger', anonymous=True)

    # Publishing
    position_pub = rospy.Publisher('chest_position', geometry_msgs.msg.Point, queue_size=1)
    velocity_pub = rospy.Publisher('chest_velocity', geometry_msgs.msg.Twist, queue_size=1)
    brake_pub = rospy.Publisher('brake_status', String, queue_size=1)
    motor_pub = rospy.Publisher('motor_status', String, queue_size=1)
    end_stop = rospy.Publisher('limits_status', String, queue_size=1)

    # Subscribing
    logger_subscriber = rospy.Subscriber("logged_info", String, logger_callback)

    # Service
    logger_control_srv = rospy.ServiceProxy('z_chest_logger', LoggerControl)

    print("Logger is ready.")

    while not rospy.is_shutdown():
        pass