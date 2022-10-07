import os
import shutil
import sys
import time
from threading import Thread

import cv2

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), '../auv_lib')))
import log  # noqa: E402 # ignore does not exist, it's in the path above ^


class VideoStreamWidget(object):
    """Camera System class

    Team member Nate Ruppert designed this

    This system is partially complex, please review the code carefully when making modifications.
    The system initializes two separate threads, one to collect frames and the other to analyze the data.
    """

    def __init__(self, src: int, led, cpu, ooi: str = '', auv: bool = True):
        """Camera System class

        Args:
            src (int): OpenCV Video Capture device [Usually 0]
            led (any): LED class to brighten area in front of camera if required
            cpu (any): CPU class to obtain temperature for frame overlay
            ooi (str): Optional setting of object of interest of ML Frame write
            auv (bool): Default of True. Whether this is being run from within
                        the AUV main code or by itself - used for testing
        """
        self.stop = False
        self.ml_frame_cap = 0
        self.lights = led
        self.cpu = cpu
        self.frame_num = 0
        self.capture = cv2.VideoCapture(src)
        self.ooi = ooi
        self.auv = auv
        if auv:
            cfg = 'camera/model/yolov4-tiny-custom.cfg'
            model = 'camera/model/yolov4-tiny-custom_last.weights'
            names = 'camera/model/obj.names'
        else:
            cfg = 'model/yolov4-tiny-custom.cfg'
            model = 'model/yolov4-tiny-custom_last.weights'
            names = 'model/obj.names'

        self.net = cv2.dnn_DetectionModel(cfg, model)
        self.net.setInputSize((640, 480))
        self.net.setInputScale(1.0 / 255)
        self.net.setInputSwapRB(True)

        # Enable Intel Neural-stick 2
        # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

        with open(names, 'rt') as f:
            self.names = f.read().rstrip('\n').split('\n')

        log.date_format('Initializing Camera System', thread='CAM_INIT')
        log.date_format('Clearing Previous Frame Data', thread='CAM_INIT')
        if auv:
            shutil.rmtree(os.path.abspath(os.path.join(os.getcwd(), 'camera/frames/')))
            os.mkdir(os.path.abspath(os.path.join(os.getcwd(), 'camera/frames/')))
        else:
            shutil.rmtree(os.path.abspath(os.path.join(os.getcwd(), 'frames/')))
            os.mkdir(os.path.abspath(os.path.join(os.getcwd(), 'frames/')))
        if ooi:
            log.date_format(f'Object of Interest has been set to {ooi}', thread='CAM_INIT')
            log.date_format('This can be changed dynamically using set_ooi() or grabbed with get_ooi()',
                            thread='CAM_INIT')
        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

        # Start ML processing non-locking
        self.mlthread = Thread(target=self.update_ml, args=())
        self.mlthread.daemon = True
        self.mlthread.start()

        time.sleep(1.1)  # pipe related print

        log.date_format('Camera Fully Initialized', thread='CAM_INIT')

    def stop_exec(self) -> None:
        """Stops threads"""
        self.stop = True

    def set_ooi(self, obj) -> None:
        """Sets the object of interest for ML purposes

        Args:
            obj (str): Object to look for - make sure class name matches exactly [eg. `Paper`]
        """
        self.ooi = obj

    def get_ooi(self) -> str:
        """Returns current object of interest"""
        return self.ooi

    def update(self) -> None:
        """Thread 1 - Camera processing and frame grab

        Returns:
            None
        """
        # Read the next frame from the stream in a different thread
        log.date_format('Thread 1 Camera Polling Initialized', thread='CAM_UPDT')
        while True:
            if self.stop:
                log.date_format('Camera Thread Stop Flag Activated - Stopping', thread='CAM_UPDT', pre='\t')
                exit(0)
            if self.capture.isOpened():
                (self.status, frame) = self.capture.read()
                self.frame = frame
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blur = cv2.mean(cv2.blur(gray, (640, 480)))[0]
                # print(blur)
                if blur > 85 and blur < 170:
                    pass
                elif blur > 170:
                    brightness = int((self.lights.get_brightness() - 5))
                    if brightness < 0:
                        brightness = 0
                    self.lights.set_brightness(brightness)
                #                     self.lights.setBrightness(0)

                else:
                    brightness = int((self.lights.get_brightness() + 5))
                    if brightness > 100:
                        brightness = 100
                    self.lights.set_brightness(brightness)
                #                     self.lights.setBrightness(0)
                self.frame_num += 1
                time.sleep(.01)
                if self.frame_num % 240 == 0:
                    frame_cap = frame
                    cv2.drawMarker(frame_cap, (640 // 2, 480 // 2), (0, 0, 255), cv2.MARKER_CROSS, 50, 2)
                    # Put cpu temp on image
                    cv2.putText(frame_cap, f'CPU Temp: {self.cpu.temp}', (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255), 2, cv2.LINE_AA, False)
                    if self.auv:
                        cv2.imwrite(f'camera/frames/frame{self.frame_num}.jpg', frame)
                    else:
                        cv2.imwrite(f'frames/frame{self.frame_num}.jpg', frame)
                    log.date_format(f'Writing Frame Capture {self.frame_num}', thread='CAM_UPDT')

    def update_ml(self) -> None:
        """Thread 2 - Machine Learning Processing thread"""

        time.sleep(1)  # Wait to guarantee buffer frame

        log.date_format('Thread 2 Machine Learning Initialized', thread='CAM_MACH')
        width_h, height_h = 640 // 2, 480 // 2
        while True:
            if self.stop:
                log.date_format('ML Thread Stop Flag Activated - Stopping', thread='CAM_MACH', pre='\t')
                exit(0)
            if self.frame_num % 6 == 0:
                frame_num = self.frame_num
                start = time.perf_counter()
                ml_frame = self.frame
                classes, confidences, boxes = self.net.detect(ml_frame, confThreshold=0.6, nmsThreshold=0.4)
                self.classes, self.confidences, self.boxes = [], [], []
                if len(classes) > 0:
                    for classId, confidence, box in zip(classes.flatten(), confidences.flatten(), boxes):
                        self.classes.append(self.names[classId])
                        self.confidences.append(confidence)
                        self.boxes.append(box)
                        left, top, width, height = box
                        class_x, class_y = (left + width // 2, top + height // 2)

                        label = f'{self.names[classId]}: {confidence:.2f} | @ {class_x, class_y}'

                        if self.ooi == self.names[classId]:
                            label_2 = 'OOI Move:'
                            if abs(class_x - width_h) < 10:
                                label_2 += ' X OK'
                            else:
                                if class_x > width_h:
                                    label_2 += f' Left {class_x - width_h} px'
                                elif class_x < width_h:
                                    label_2 += f' Right {width_h - class_x} px'

                            if abs(class_y - height_h) < 10:
                                label_2 += ' Y OK'
                            else:
                                if class_y > height_h:
                                    label_2 += f' Down {class_y - height_h} px'
                                elif class_y < height_h:
                                    label_2 += f' Up {height_h - class_y} px'
                            label_size_2, baseline_2 = cv2.getTextSize(label_2, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                            cv2.putText(ml_frame, label_2, (640 - label_size_2[0] - 5, 480 - label_size_2[1]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                        label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        top = max(top, label_size[1])
                        cv2.rectangle(ml_frame, box, color=(0, 255, 0), thickness=3)
                        cv2.rectangle(ml_frame, (left, top - label_size[1]), (left + label_size[0], top + baseline),
                                      (255, 255, 255), cv2.FILLED)
                        cv2.drawMarker(ml_frame, (class_x, class_y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
                        cv2.putText(ml_frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                #                 print(self.classes, self.confidences, self.boxes)
                cv2.drawMarker(ml_frame, (width_h, height_h), (0, 0, 255), cv2.MARKER_CROSS, 50, 2)
                # Put cpu temp on image
                cv2.putText(ml_frame, f'CPU Temp: {self.cpu.temp}', (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2, cv2.LINE_AA, False)
                self.ml_frame = ml_frame  # once complete, update frame for user visualization
                if self.auv:
                    cv2.imwrite(f'camera/frames/frame{frame_num}_ML.jpg', ml_frame)
                else:
                    cv2.imwrite(f'frames/frame{frame_num}_ML.jpg', ml_frame)

                log.date_format(f'ML Processing on frame {frame_num} took Thread 2 {time.perf_counter() - start:.2f}s',
                                thread='CAM_MACH')
                self.ml_frame_cap = frame_num

    def show_frame(self) -> None:
        """Warn: Does not work unless GUI is enabled on Pi - Do not use"""
        # Display frames in main program
        cv2.imshow('frame', self.frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)

    def show_ml_frame(self) -> None:
        """Warn: Does not work unless GUI is enabled on Pi - Do not use"""
        cv2.imshow('ml_frame', self.ml_frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)


if __name__ == '__main__':
    print('Running Camera Code test [Warn: cv2.error for cv2.imshow if no GUI/running via terminal]')

    # Library imports of led code just for code test ONLY
    import cpu_read
    import led
    import pigpio

    pi = pigpio.pi()
    leds = led.LEDs(pi, [23])
    cpu = cpu_read.CPURead()
    camera = VideoStreamWidget(0, leds, cpu, auv=False, ooi='Paper')  # verify gpio pin for led
    try:
        while True:
            try:
                pass
                # camera.show_frame()
                # camera.show_ml_frame()
            except AttributeError:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        cpu.stop_exec()
        camera.stop_exec()
        time.sleep(5)
        leds.set_brightness(0)
        leds.close_gpio()
        exit(0)
