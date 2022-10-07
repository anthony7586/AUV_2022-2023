import time
from subprocess import PIPE, Popen
from threading import Thread

import log


class CPURead:
    """
    Class designed to read the Raspberry Pi CPU Temperature using `vcgencmd`

    Team member Nate Ruppert designed this
    """

    def __init__(self):
        self.stop = False
        self.temp = self.get_cpu_temperature()

        # Start the thread to read CPU
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def stop_exec(self) -> None:
        """Terminates `update()` thread"""
        self.stop = True

    def get_cpu_temperature(self) -> str:
        """Collects CPU temperature of the pi using `vcgencmd measure_temp`

        Returns:
            cpu_temp (bytes): In format `80.0'C`
        """
        process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
        output, _error = process.communicate()
        res = str(output).split('=')[1]
        res = res.replace(r'\n"', '')
        return res

    def update(self) -> None:
        """Threaded collection of CPU Temp every 3 seconds

        Outputs to sys.stdout the results

        If stop of thread is desired, please run stop_exec which will terminate the thread gracefully
        """
        log.date_format('Initializing CPU Temp Read-out', thread='CPU_READ')
        while True:
            if self.stop:
                log.date_format('Thread Stop Flag Activated - Stopping', thread='CPU_READ', pre='\t')
                exit(0)
            time.sleep(3)
            temp = self.get_cpu_temperature()
            self.temp = temp
            temp_val = float(temp.split('\'')[0])
            if 80.0 < temp_val < 85.0:
                temp += f' WARN THROTTLE [Thermal Headroom: {85.0 - temp_val:+.1f}C]'
            elif temp_val > 85.0:
                temp += f' WARN OVERHEAT [Thermal Headroom: {85.0 - temp_val:+.1f}C]'
            else:
                temp += f' SAFE NO ISSUE [Thermal Headroom: {85.0 - temp_val:+.1f}C]'
            log.date_format(f'CPU Temp: {temp}', thread='CPU_READ')


if __name__ == '__main__':
    cpu = CPURead()
    while True:
        pass
