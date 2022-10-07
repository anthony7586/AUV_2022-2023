import signal  # note: only works on linux :)
import time
from typing import List

import adafruit_bno055
import board
import numpy as np
import pigpio

# Project library imports
from auv_lib import cpu_read, gripper, led, log, thruster
from camera import camera_non_locking

camera_id = -1
# [Front L, Front R, Back L, Back R, Up L, Up R]
thruster_pins = [17, 6, 4, 22, 27, 5]
led_pin = [23]
gripper_pin = [18]

obj = {1: "gman",
       2: "bootlegger",
       3: "badge",
       4: "tommygun",
       5: "wiskey",
       6: "barrel",
       7: "phone",
       8: "notepad",
       9: "axe",
       10: "bottle",
       11: "dollarsign",
       12: "cell phone"
       }


class AUV:
    """AUV System Code

    Mostly designed by Nate, with Functions to control thrusters by Henry

    Code is honestly self-explanatory - follow the print statements, and you
    will understand what the code is doing. Code is extremely 'safe', as in
    no matter what state the code ends up in, it should gracefully exit thrusters, led, gripper
    in a way that prevents unnecessary current drain or leftover components being on after completion
    of execution.
    """

    def __init__(self, thruster_pin: List[int], leds_pin: List[int], grip_pin: List[int], camera_index: int):
        """AUV System Code

        Args:
            thruster_pin (List[int]): List of Thruster pins
            leds_pin (List[int]): List of LED pins (currently 1)
            grip_pin (List[int]): List of Gripper pins (currently 1)
            camera_index (int): Camera index (Usually 0)
        """
        log.date_format('Beginning System Activation', thread='AUV_INIT')

        self.pi = pigpio.pi()
        log.date_format('Raspberry Pi GPIO Initialized', thread='AUV_INIT')
        time.sleep(1)  # let pi initialize
        # Thruster, LED, Gripper, Camera init
        self.thruster = thruster.Thrusters(self.pi, thruster_pin)
        log.date_format('Thrusters Initialized', thread='AUV_INIT')
        self.led = led.LEDs(self.pi, leds_pin)
        log.date_format('LEDs Initialized', thread='AUV_INIT')
        self.gripper = gripper.Gripper(self.pi, grip_pin)
        log.date_format('Gripper Initialized', thread='AUV_INIT')
        self.cpu = cpu_read.CPURead()
        time.sleep(0.5)
        log.date_format('CPU Temperature Read Initialized', thread='AUV_INIT')
        self.camera = camera_non_locking.VideoStreamWidget(camera_index, self.led, self.cpu, ooi='Paper')
        log.date_format('Camera Initialized', thread='AUV_INIT')
        # Activation of i2c and the gyro
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        log.date_format('Gyro Sensor Successfully Activated', thread='AUV_INIT')
        time.sleep(1)

        # Flash method to indicate system activation success
        log.date_format('Flashing LEDs to indicate Success [3x] + Activating Thrusters', thread='AUV_INIT')
        self.thruster.open_gpio()
        for _ in range(0, 3):  # 3 times
            self.led.set_brightness(80)
            time.sleep(0.1)
            self.led.set_brightness(0)
            time.sleep(0.1)

        log.date_format('System Activation Complete', thread='AUV_INIT')

    # X-axis (Forward and Reverse)
    # Direction 1 is forward
    # Direction -1 is reverse
    # Direction 0 is stop
    def surge(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates all four forward/reverse thrusters

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
        self.thruster.frontthrusters(direction, speed, ramp)
        self.thruster.backthrusters(direction, speed, ramp)

    # Z-axis (Up and Down)
    # Direction 1 is up
    # Direction -1 is down
    # Direction 0 is stop
    # [Front L, Front R, Back L, Back R, Up L, Up R]
    def heave(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates vertical thrusters

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
        self.thruster.verticalthrusters(direction, speed, ramp)

    # Rotation around the Z-axis
    # Direction 1 is rotating counterclockwise
    # Direction -1 is rotating clockwise
    # Direction 0 is stop
    def yaw(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates thrusters in a way that allows for rotational movement

        Args:
            direction (int): Direction of move [0 is hold, 1 is counterclockwise, -1 is clockwise]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        self.thruster.individualthruster(self.thruster.pins[0], (-1) * direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[1], direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[2], direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[3], (-1) * direction, speed, ramp)

    # Rotation around the X-axis
    # Direction 1 is rotating counterclockwise
    # Direction -1 is rotating clockwise
    # Direction 0 is stop
    def roll(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates vertical thrusters in a way that allows for rotational movement

        Args:
            direction (int): Direction of move [0 is hold, 1 is counterclockwise, -1 is clockwise]
            speed (int): Speed [0-100]% - Be careful
            ramp (bool): Whether to slowly increase speed or not

        Returns:
            None

        Warnings:
            Ramp does not work as intended - Henry did not test before implementation

            See Nate's variation in `test_thrust()` for example of how this will have to be redone
        """
        self.thruster.individualthruster(self.thruster.pins[4], direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[5], (-1) * direction, speed, ramp)

    # Strafe left forward and reverse
    # Direction 1 is Strafe left forward
    # Direction 0 is stop
    # Direction -1 is Strafe left reverse
    def strafeleft(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates thrusters to allow for strafing left ( no idea what this means )

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
        self.thruster.individualthruster(self.thruster.pins[1], direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[2], (-1) * direction, speed, ramp)

    # Strafe left forward and reverse
    # Direction 1 is Strafe right forward
    # Direction 0 is stop
    # Direction -1 is Strafe right reverse
    def straferight(self, direction: int, speed: int, ramp: bool) -> None:
        """Activates thrusters to allow for strafing right ( no idea what this means )

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
        self.thruster.individualthruster(self.thruster.pins[0], direction, speed, ramp)
        self.thruster.individualthruster(self.thruster.pins[3], (-1) * direction, speed, ramp)

    def level(self) -> None:
        """Supposedly levels out the AUV - im not touching this"""
        _, speed = self.thruster.get_thruster_speed()
        _, _, _, _, Up_L, Up_R = speed

        islevel = True
        while (islevel):
            _, _, z = self.sensor.euler
            print(self.sensor.gyro, np.pi)
            _, _, gyro_z = [element / 2 * 360 for element in self.sensor.gyro]
            print(gyro_z)

            if z > -180 and z < -5:
                percent = abs(z / 75)
                self.yaw(-1, int(15 * percent), True)

            elif z < 180 and z > 5:
                percent = abs(z / 70)
                self.yaw(1, int(15 * percent), True)

            elif z < 5 and abs(gyro_z) > .1:
                percent = abs(z / 10)
                self.yaw(-1, int(5 * percent), True)

            elif z > -5 and abs(gyro_z) > .1:
                percent = abs(z / 10)
                self.yaw(-1, int(5 * percent), True)

            elif z < 5 and z > -5 and abs(gyro_z) <= .1:
                islevel = False
        self.thruster.pi.set_servo_pulsewidth(self.thruster.pins[4], Up_L)
        self.thruster.pi.set_servo_pulsewidth(self.thruster.pins[5], Up_R)


def end(*args):  # processes linux signal interrupts to forcibly stop execution
    """Ends system execution gracefully after receiving sigterm or sigint"""
    log.date_format('Received System Termination Signal - Processing', thread='SYS_EXIT',
                    pre='\n')  # go to line after the ^C

    # Thruster quit - not threaded
    auv.thruster.test_thrust(0, 0, 0, 0, 0, timer=0)
    auv.thruster.test_down(0, 0, timer=0)
    auv.thruster.close_gpio()
    log.date_format('All Thrusters Disabled', thread='SYS_EXIT', pre='\t')

    # LEDs quit - not threaded
    auv.led.set_brightness(0)
    auv.led.close_gpio()
    log.date_format('LEDs Disabled', thread='SYS_EXIT', pre='\t')

    # Gripper quit - not threaded
    auv.gripper.close_gpio()
    log.date_format('Gripper Disabled', thread='SYS_EXIT', pre='\t')

    # Camera - 2 threads to force close
    auv.camera.stop_exec()
    log.date_format('Sent Camera Thread 1 and 2 Quit Signal', thread='SYS_EXIT', pre='\t')

    # CPU temp - 1 thread to force close
    auv.cpu.stop_exec()
    log.date_format('Sent CPU Temp Thread Quit Signal', thread='SYS_EXIT', pre='\t')

    time.sleep(4)  # wait a bit to ensure thread close

    # Done
    log.date_format('System Termination Complete', thread='SYS_EXIT')
    exit(0)


tracked_frame = 0  # keeps track if processing has already completed on available frame
auv = AUV(thruster_pins, led_pin, gripper_pin, camera_id)
signal.signal(signal.SIGTERM, end)
signal.signal(signal.SIGINT, end)

while True:
    try:
        if auv.camera.ml_frame_cap != tracked_frame and auv.camera.classes:
            tracked_frame = auv.camera.ml_frame_cap
            # log.date_format(f'Found ml capture on frame {tracked_frame}', thread='AUV_MAIN')
            # print(auv.camera.classes, auv.camera.confidences, auv.camera.boxes)
            classes, confidences, boxes = auv.camera.classes, auv.camera.confidences, auv.camera.boxes
            for cl, conf, box in zip(classes, confidences, boxes):
                if cl == 'Person':
                    log.date_format(
                        f'Frame {tracked_frame}: Found OOI {cl} @ Confidence of {conf * 100:.2f}% [Box {box}]',
                        thread='AUV_MAIN')
                    x, y = round(box[0] + box[2] / 2), round(box[1] + box[3] / 2)
                    log.date_format(f'Box is at: {x} / {y}', thread='AUV_MAIN', pre='\t')

                    x_res, y_res = 640, 480
                    x_mid, y_mid = int(x_res * .5), int(y_res * .5)

                    # if x < int(x_mid - x_res * .125):
                    #     print("AUV is Below Target")
                    #     auv.heave(1, 20, False)
                    # elif x > int(x_mid + x_res * .125):
                    #     print("AUV is Above Target")
                    #     auv.heave(-1, 20, False)
                    # else:
                    #     print("AUV is Vertically Center")
                    #     auv.heave(0, 0, False)
                    if x < int(y_mid - y_res * .125):
                        log.date_format("AUV is located Left of Target", thread='AUV_MAIN', pre='\t')
                        auv.yaw(1, 20, False)
                    elif x > int(y_mid + y_res * .125):
                        log.date_format("AUV is located Right of Target", thread='AUV_MAIN', pre='\t')
                        auv.yaw(-1, 20, False)
                    else:
                        log.date_format("AUV is Horizontally center on target", thread='AUV_MAIN', pre='\t')
                        auv.yaw(0, 0, False)
                else:
                    log.date_format(f'Frame {tracked_frame}: Found {cl} @ Confidence of {conf * 100:.2f}% [Box {box}]',
                                    thread='AUV_MAIN')
            time.sleep(1)
        else:
            auv.heave(0, 0, False)
            auv.yaw(0, 0, False)
            time.sleep(1)
    except AttributeError:
        # print('no')
        time.sleep(4)
