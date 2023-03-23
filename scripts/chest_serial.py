#!/usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
from std_msgs.msg import *
from gopher_ros_clearcore.srv import *


class GopherSerial:
    def __init__(self):
        self.logger_pub = rospy.Publisher("logged_info", String, queue_size=1)
        self.serial_write_src = rospy.Service('serial_write', SerialWrite, self.serial_write_handler)
        self.serialPort = None
        self.serial_setup()
    
    
    # Serial connection setup function
    def serial_setup(self, device_name="Teknic ClearCore", baudrate=115200, timeout=0.1):
        ports = list(serial.tools.list_ports.comports())

        port = ""

        # Finds a port name for the Teknic ClearCore
        for p in ports:
            if(device_name in p):
                port = str(p).split("-")[0].strip()
        
        self.serialPort = serial.Serial(port, baudrate, timeout=timeout)


    # Service handler
    def serial_write_handler(self, req):
        self.serial_write(req.command)

        # TODO: Service response
        response = True

        return response

    
    # Write to the serial port
    def serial_write(self, data):
        self.serialPort.write(data.encode())


    # Read from the serial port
    def serial_read(self):
        data = self.serialPort.readline().decode()
       
        # If any serial data available
        if len(data) > 0:
            
            data = data.split("_")

            command = data[0]

            # Publish logger data
            if command == "logger":
                self.logger_pub.publish(data[1])


# This function is called when the node is shutting down
def node_shutdown():
    # Deactivate logger
    serial.serial_write("logger_off_")

    # Stop chest motion
    serial.serial_write("vm_0.0_")
    print("Shutting down the serial connection...")
        
        
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('chest_serial')
    rospy.on_shutdown(node_shutdown)
    
    serial = GopherSerial() 

    print("Serial connection is setup.")
    
    while not rospy.is_shutdown():
        serial.serial_read()