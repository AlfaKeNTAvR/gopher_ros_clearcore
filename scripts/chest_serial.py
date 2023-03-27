#!/usr/bin/env python
"""
A ROS node to communicate with the Teknic ClearCore via serial connection.

Author(s):  
    1. Yveder Joseph (ygjoseph@wpi.edu)
    2. Nikita Boguslavskii (bognik3@gmail.com)
"""

import rospy
import serial
import serial.tools.list_ports

from std_msgs.msg import String

from gopher_ros_clearcore.srv import SerialWrite


class GopherSerial:
    """Provides an interface to communicate via serial connection.

    Attributes:
        logger_pub (rospy.Publisher): A ROS publisher to publish logger data
            received from Teknic ClearCore controller.
        serial_write_src (rospy.Service): A ROS service to write data to Teknic 
            ClearCore controller via serial port.
        serialPort (serial.Serial): The serial port instance used to
            communicate with Teknic ClearCore.
    """

    def __init__(self):
        self.logger_pub = rospy.Publisher(
            'logged_info',
            String,
            queue_size=1,
        )
        self.serial_write_src = rospy.Service(
            'serial_write',
            SerialWrite,
            self.serial_write_handler,
        )
        self.serialPort = None
        self.serial_setup()

    def serial_setup(
        self,
        device_name='Teknic ClearCore',
        baudrate=115200,
        timeout=0.1,
    ):
        """Sets up the serial port.

        Args:
            device_name (str): The name of the device to connect to.
            baudrate (int): The baudrate to use for the serial connection.
            timeout (float): The timeout for the serial connection.
        """

        ports = list(serial.tools.list_ports.comports())

        port = ''

        # Finds a port name for the Teknic ClearCore.
        for p in ports:
            if (device_name in p):
                port = str(p).split('-')[0].strip()

        self.serialPort = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
        )

    def serial_write(self, data):
        """Writes data to the serial port.

        Args:
            data (str): The data to be written to the serial port.
        """

        self.serialPort.write(data.encode())

    def serial_read(self):
        """Reads data from the serial port. 
        
        Publishes logger data to the logged_info topic.
        """

        data = self.serialPort.readline().decode()

        if len(data) > 0:  # If any serial data available.

            data = data.split('_')
            command = data[0]

            # Publish logger data
            if command == 'logger':
                self.logger_pub.publish(data[1])

    def serial_write_handler(self, req):
        """Handles serial write ROS service requests.

        Args:
            req (SerialWriteRequest): The request object containing the command 
            to be written to the serial port.

        Returns:
            bool: A boolean value indicating whether the request was successful.
        """

        self.serial_write(req.command)

        # TODO: Service response
        response = True

        return response


# Is called when the node is shutting down.
def node_shutdown():
    # Deactivate the logger.
    serial.serial_write('logger_off_')

    # Stop any chest motion.
    serial.serial_write('vm_0.0_')
    print('Shutting down the serial connection...')


def main():
    # Initialize the node
    rospy.init_node('chest_serial')
    rospy.on_shutdown(node_shutdown)

    serial = GopherSerial()

    print('Serial connection is setup.')

    while not rospy.is_shutdown():
        serial.serial_read()


if __name__ == '__main__':
    main()