#!/usr/bin/env python
"""

"""

import rospy
import numpy as np

from geometry_msgs.msg import (Twist)

from oculus.msg import (
    ControllerButtons,
    ControllerJoystick,
)
from gopher_ros_clearcore.srv import (
    Homing,
    Stop,
)


class OculusChestMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='right',
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        # # Private variables:
        self.__oculus_joystick = ControllerJoystick()
        self.__oculus_buttons = ControllerButtons()

        self.__joystick_button_state = 0

        # # Public variables:

        # # ROS node:
        rospy.init_node('oculus_chest_mapping')
        rospy.on_shutdown(self.__node_shutdown)

        # # Service provider:

        # # Service subscriber:
        self.__chest_home = rospy.ServiceProxy(
            'z_chest_home',
            Homing,
        )
        self.__chest_stop = rospy.ServiceProxy(
            'z_chest_stop',
            Stop,
        )

        # # Topic publisher:
        self.__chest_velocity = rospy.Publisher(
            'z_chest_vel',
            Twist,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

    # # Private methods:
    def __node_shutdown(self):
        """
        
        """

        print('\nNode is shutting down...\n')

        self.__chest_stop()

        print('\nNode is shut down.\n')

    def __map_chest(self):
        """
        
        """

        chest_velolicity = 0.0

        if abs(self.__oculus_joystick.position_y) > 0.05:  # Noisy joystick.
            chest_velolicity = np.interp(
                round(self.__oculus_joystick.position_y, 4),
                [-1.0, 1.0],
                [-0.8, 0.8],
            )

        velocity_message = Twist()
        velocity_message.linear.z = chest_velolicity
        self.__chest_velocity.publish(velocity_message)

    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Homing is activated.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__chest_home(True)
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 0

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__joystick_button_state_machine()
        self.__map_chest()


def main():
    """
    
    """

    chest_mapping = OculusChestMapping(controller_side='right')

    print('\nOculus-chest mapping is ready.\n')

    while not rospy.is_shutdown():
        chest_mapping.main_loop()


if __name__ == '__main__':
    main()