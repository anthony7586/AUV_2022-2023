import time
from typing import List

import numpy as np
import pigpio


class LEDs:
    """Class to control LEDs

    Team members Nate Ruppert and Henry Lin designed this

    Current system includes two LEDs wired directly together, meaning only one pin controls both.
    """

    def __init__(self, pig, pins: List[int]):
        """Class to control LEDs

        Args:
            pig (obj): PIGPIO connection to Raspberry Pi GPIO
            pins (List[int]): List of pins to use (For LEDs there currently exists 1 in the list such as `[1]`)
        """
        self.pi = pig
        self.led_pin = pins

        # Setting up for conversion from 0 to 100 %
        self.duty_cycle = [1100, 1500, 1900]
        self.lights = np.linspace(self.duty_cycle[0], self.duty_cycle[2], 101)
        self.open_gpio()

    def open_gpio(self) -> None:
        """Opens LED GPIO"""
        self.pi.set_servo_pulsewidth(self.led_pin[0], 1100)

    def close_gpio(self) -> None:
        """Closes LED GPIO"""
        self.pi.set_servo_pulsewidth(self.led_pin[0], 0)

    def set_brightness(self, brightness: int) -> None:
        """Sets brightness of LEDs

        Args:
            brightness (int): Percent of LED brightness to set in form `80` or `0` or `50` %
        """
        self.pi.set_servo_pulsewidth(self.led_pin[0], self.lights[brightness])

    def get_brightness(self) -> int:
        """Gets brightness of LEDs

        Returns:
            brightness (int): Brightness in Percent of LEDs
        """
        return int((self.pi.get_servo_pulsewidth(self.led_pin[0]) - 1100) / 800 * 100)


if __name__ == '__main__':
    pi = pigpio.pi()
    led_pin = [23]  # please verify
    led = LEDs(pi, led_pin)
    print('Starting LED test')
    for i in range(0, 10):
        led.set_brightness(i * 10)
        time.sleep(1)
    led.set_brightness(0)
