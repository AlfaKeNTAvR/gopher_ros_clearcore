#!/usr/bin/env python
"""
A ROS node to communicate with Teknic ClearCore via serial connection.

Author(s):  
    1. Yveder Joseph (ygjoseph@wpi.edu), Worcester Polytechnic Institute (WPI),
       2023.
    2. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
"""

# # Standart libraries:
import rospy
import serial
import serial.tools.list_ports

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    String,
)

# # Third party messages and services:
from gopher_ros_clearcore.srv import (SerialWrite)


class ChestSerial:
    """Provides an interface to communicate via serial connection.

    Attributes:
        logger_info (rospy.Publisher): A ROS publisher to publish logger data
            received from Teknic ClearCore controller.
        serial_write (rospy.Service): A ROS service to write data to Teknic 
            ClearCore controller via serial port.
        serial_port (serial.Serial): The serial port instance used to
            communicate with Teknic ClearCore.
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__serial_port = None

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            # 'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        # self.__dependency_status_topics['<dependency_node_name>'] = (
        #     rospy.Subscriber(
        #         f'/<dependency_node_name>/is_initialized',
        #         Bool,
        #         self.__dependency_name_callback,
        #     )
        # )

        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/serial_write',
            SerialWrite,
            self.__serial_write_handler,
        )

        # # Service subscriber:

        # # Topic publisher:
        self.__logger_info = rospy.Publisher(
            '/chest_logger/logger_info',
            String,
            queue_size=1,
        )

        # # Topic subscriber:

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    # def __dependency_name_callback(self, message):
    #     """Monitors <node_name>/is_initialized topic.

    #     """

    #     self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:
    def __serial_write_handler(self, request):
        """Handles serial write ROS service requests.

        Args:
            req (SerialWriteRequest): The request object containing the command 
            to be written to the serial port.

        Returns:
            bool: A boolean value indicating whether the request was successful.
        """

        self.__serial_write(request.command)

        # TODO: Service response
        response = True

        return response

    # # Topic callbacks:

    # # Timer callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized and self.__serial_port):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                # Deactivate the logger.
                self.__serial_write('logger_off_')

                # Stop any chest motion.
                self.__serial_write('vm_0.0_')

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __serial_setup(
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

        self.__serial_port = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
        )

    def __serial_write(self, data):
        """Writes data to the serial port.

        Args:
            data (str): The data to be written to the serial port.
        """

        self.__serial_port.write(data.encode())

    def __serial_read(self):
        """Reads data from the serial port. 
        
        Publishes logger data to the logged_info topic.
        """

        data = self.__serial_port.readline().decode()

        if len(data) > 0:  # If any serial data available.

            data = data.split('_')
            command = data[0]

            # Publish logger data
            if command == 'logger':
                self.__logger_info.publish(data[1])

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        if not self.__serial_port:
            self.__serial_setup()

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        self.__serial_read()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # Deactivate the logger.
        self.__serial_write('logger_off_')

        # Stop any chest motion.
        self.__serial_write('vm_0.0_')

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'chest_serial',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    class_instance = ChestSerial(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
