import time

import numpy as np
import pigpio


class Thrusters:
    """Class to control thrusters

    Team members Nate Ruppert and Henry Lin designed this

    Thruster functions are iffy at best, Henry was not able to test thrusters directly at the time. Future
    team will be required to test and/or redesign all functions as needed to support future efforts.
    """

    def __init__(self, pig, pins):
        """Class to control thrusters

        Args:
            pig (obj): PIGPIO connection to Raspberry Pi GPIO
            pins (List[int]): List of pins to use [Expected List size of 6]
        """
        # Expecting 6 pins. Assuming the order is [Front L, Front R, Back L, Back R, Up L, Up R]
        # GPIO.cleanup()
        self.pi = pig
        self.pins = pins

        # Setting up for conversion from 0 - 100 % for 11%-15% and 15%-19% duty cycle
        self.dutycycle = [1100, 1500, 1900]
        self.forwardspeed = np.linspace(self.dutycycle[1], self.dutycycle[2], 101)
        # Reversing because speed control is a parabola where the center is 15%
        self.reversespeed = np.linspace(self.dutycycle[0], self.dutycycle[1], 101)
        self.reversespeed = np.flip(self.reversespeed)

    def close_gpio(self) -> None:
        """Closes Thruster GPIO - This is highly recommended for safety and power reasons"""
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin, 0)

    def open_gpio(self) -> None:
        """Opens Thruster GPIO - Please remember to close using `close_gpio()`"""
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin, 1500)  # 1500 activates

    # Provide a direction and speed from 0 - 100
    # 1 being forward, 0 being reverse
    def frontthrusters(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates front two thrusters

        Args:
            direction (int): Direction of move [0 is hold, 1 is forward, -1 is reverse]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        self.individualthruster(self.pins[0], direction, speed, ramp)
        self.individualthruster(self.pins[1], direction, speed, ramp)

    def backthrusters(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates back two thrusters

        Args:
            direction (int): Direction of move [0 is hold, 1 is forward, -1 is reverse]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        self.individualthruster(self.pins[2], direction, speed, ramp)
        self.individualthruster(self.pins[3], direction, speed, ramp)

    def verticalthrusters(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates vertical two thrusters

        Args:
            direction (int): Direction of move [0 is hold, 1 is up, -1 is down]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        self.individualthruster(self.pins[4], (-1) * direction, speed, ramp)
        self.individualthruster(self.pins[5], (-1) * direction, speed, ramp)

    def individualthruster(self, pin: int, direction: int, speed: int, ramp: bool) -> None:
        """Activates a thruster

        Args:
            pin (int): Pin to use of thruster
            direction (int): Direction of move [0 is hold, 1 is forward, -1 is reverse]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        _, speeds = self.get_thruster_speed()
        if ramp:
            location = self.pins.index(pin)
            start = speeds[location]
            end = speed
            segments = abs(int((start - end) / 10))
            if segments == 0:
                segments = 1
            # Range(current speed, end speed, semgments in speed)
            print(start, end, segments)
            if direction == 1:
                for i in range(start + segments, end + segments, segments):
                    self.pi.set_servo_pulsewidth(pin, self.forwardspeed[i])
                    time.sleep(.05)
            elif direction == -1:
                for i in range(start + segments, end + segments, segments):
                    self.pi.set_servo_pulsewidth(pin, self.reversespeed[i])
                    time.sleep(.05)
            elif direction == 0:
                self.pi.set_servo_pulsewidth(pin, 1500)

        else:
            if direction == 1:
                self.pi.set_servo_pulsewidth(pin, self.forwardspeed[speed])
            elif direction == -1:
                self.pi.set_servo_pulsewidth(pin, self.reversespeed[speed])
            elif direction == 0:
                self.pi.set_servo_pulsewidth(pin, 1500)

    def get_thruster_speed(self):
        """Provides thruster speed for all thrusters

        This has likely never been tested - use if needed or rewrite as needed

        Returns:
            direction (List[int]): Direction of thrusters at current moment
            speeds (List[int]): Speed of thrusters at current moment
        """
        speeds = []
        direction = []
        for pin in self.pins:
            vector = int((self.pi.get_servo_pulsewidth(pin) - self.dutycycle[1]) / self.dutycycle[2])
            if vector < 0:
                direction.append(-1)
            elif vector == 0:
                direction.append(0)
            else:
                direction.append(1)
            speeds.append(abs(vector))
        return direction, speeds

    def test_thrust(self, speed: int, speed2: int, speed3: int, speed4: int, dir: int, safe: bool = True,
                    ramp: bool = False, timer: int = 20) -> None:
        """Tests all four thrusters designed for forward/reverse movement

        Args:
            speed (int): Left Front thruster speed
            speed2 (int): Right Front thruster speed
            speed3 (int): Back Left thruster speed
            speed4 (int): Back Right thruster speed
            dir (int): Direction of move for all thrusters
            safe (bool): Safety feature to auto-stop thrusters in event of timeout
            ramp (bool): Whether to ramp thruster speeds to the value over the course of 2.5s
            timer (int): Timeout of system - defaults to 20s

        Returns:
            None
        """
        if ramp:
            for i in range(0, 50):
                self.individualthruster(self.pins[0], dir, int(speed * i / 50), False)
                self.individualthruster(self.pins[1], dir, int(speed2 * i / 50), False)
                self.individualthruster(self.pins[2], -dir, int(speed2 * i / 50), False)
                self.individualthruster(self.pins[3], -dir, int(speed3 * i / 50), False)
                time.sleep(0.05)
        else:
            self.individualthruster(self.pins[0], dir, speed, False)
            self.individualthruster(self.pins[1], dir, speed2, False)
            self.individualthruster(self.pins[2], -dir, speed3, False)
            self.individualthruster(self.pins[3], -dir, speed4, False)
        if safe:
            try:
                time.sleep(timer)
            except KeyboardInterrupt:
                pass
            finally:
                self.frontthrusters(0, 0, False)
                self.backthrusters(0, 0, False)

    def test_down(self, speed: int, dir: int, timer: int = 20) -> None:
        """Similar to `test_thrust` but for vertical thrusters only. More rudimentary than `test_thrust`.

        Args:
            speed (int): Thrusters speed
            dir (int): Direction of move for all thrusters
            timer (int): Timeout of system - defaults to 20s

        Returns:
            None
        """
        try:
            self.verticalthrusters(dir, speed, False)
            time.sleep(timer)
        except KeyboardInterrupt:
            pass
        finally:
            self.verticalthrusters(0, speed, False)


if __name__ == '__main__':
    pi = pigpio.pi()
    #               [Front L, Front R, Back L, Back R, Up L, Up R]
    thruster_pin = [17, 6, 4, 22, 27, 5]
    thruster = Thrusters(pi, thruster_pin)
    thruster.open_gpio()
    time.sleep(2)
    print('Forward 10% Test: Ctrl+C to stop anytime')
    print('Note: For interactivity, run file with -i.')
    print('Function is test_thrust(speed0-4[int%], Direction[t/f], Ramp[t/f])')
    thruster.test_thrust(20, 20, 20, 20, 1)
    thruster.close_gpio()
