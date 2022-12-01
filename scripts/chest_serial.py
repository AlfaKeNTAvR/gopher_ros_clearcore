#!/usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
from std_msgs.msg import *
from gopher_ros_clearcore.srv import *


class GopherSerial:
    def __init__(self):
        self.logger_pub = rospy.Publisher("logged_info", String, queue_size=1)
        self.serial_write_src = rospy.Service('serial_write', SerialWrite, self.serial_write)
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


    # Write to the serial port
    def serial_write(self, req):
         self.serialPort.write(req.command.encode())


    # Read from the serial port
    def serial_read(self):
        data = self.serialPort.readline().decode("utf-8").strip()
        if len(data) > 0:
            self.logger_pub.publish(data)


    # Actions when the node dies
    def on_shutdown(self):
        self.serial_write("vm_0.0_")
        print("Shutting down the serial connection...")
        
        
if __name__ == '__main__':
    rospy.init_node('chest_serial')
    serial = GopherSerial()   
    
    while not rospy.is_shutdown():
        serial.serial_read()

    # Stop the chest motion
    serial.on_shutdown()