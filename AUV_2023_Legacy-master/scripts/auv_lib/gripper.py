from typing import List

import numpy as np
import pigpio


# general note: this code is wrong - henry originally thought microsecond would translate
# into % open of gripper - that's not correct. future team will require fixing this (please see BlueRobotics site)
class Gripper:
    """Gripper System Class to control Gripper

    Team Member Henry Lin designed most of this code - it does not work as originally intended

    Henry originally thought microsecond write in a linear fashion would translate to % open of gripper but that is
    not correct. You as future team will have to correct this - please use BlueRobotics site/forum to determine fix.
    """

    def __init__(self, pig, pins: List[int]):
        """Gripper System Class to control Gripper

        Team Member Henry Lin designed this code - it does not work as originally intended

        Args:
            pig (obj): PIGPIO connection to Raspberry Pi GPIO
            pins (List[int]): List of pins to use (For Gripper there currently exists 1 in the list such as `[1]`)
        """
        # Expecting 1 pins, in no particular order
        self.pi = pig
        self.pins = pins

        # Setting up for conversion from 0 - 100 % for 11%-15% and 15%-19% duty cycle
        self.dutycycle = [1100, 1500, 1900]
        self.openness = np.linspace(self.dutycycle[0], self.dutycycle[2], 101)
        self.open_gpio()

    def open_gpio(self) -> None:
        """Opens Gripper GPIO"""
        self.pi.set_servo_pulsewidth(self.pins[0], 1500)  # no movement state

    def close_gpio(self) -> None:
        """Closes Gripper GPIO"""
        self.pi.set_servo_pulsewidth(self.pins[0], 0)

    def setOpenness(self, percent: int) -> None:
        """Once again see Gripper class documentation on why this is wrong - please fix"""
        self.pi.set_servo_pulsewidth(self.pins[0], self.openness[percent])
        self.close_gpio()  # always close pwm after move


if __name__ == '__main__':
    pi = pigpio.pi()
    gripper_pin = [18]
    gripper = Gripper(pi, gripper_pin)
    gripper.setOpenness(100)
