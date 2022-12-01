#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
import geometry_msgs.msg 
import serial
import json

def logger():
    position_pub = rospy.Publisher('chest_position', geometry_msgs.msg.Point, queue_size=10)
    velocity_pub = rospy.Publisher('chest_velocity', geometry_msgs.msg.Twist, queue_size=10)
    brake_pub = rospy.Publisher('brake_status', String, queue_size=10)
    motor_pub = rospy.Publisher('motor_status', String, queue_size=10)
    end_stop = rospy.Publisher('limits_status', String, queue_size=10)
    
    rospy.init_node('logger', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Connects to clearCore via Serial Communication
    clearCore = serial.Serial('/dev/ttyACM0', 115200,timeout=.1)
    while not rospy.is_shutdown():
        data = clearCore.readline().decode("utf-8").strip()

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

        rate.sleep()

if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException:
        pass