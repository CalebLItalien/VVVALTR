#!/usr/bin/python
# Written by James Boggs

import sys
from subprocess import Popen, PIPE, call, check_output, CalledProcessError
from threading import Thread
from time import sleep

import cv2

from tensTracking import TensTracker, Point

ERROR_CODES = {0: 'Success', 1: 'Failure'}
EXP_FIN_SIG = [255, 255]  # signal that the experiment finished
EXP_FIN_SIG_BIG = [255, 255, 0, 0]
RESTART_SIGNALS = [[1, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 1]]


class TensegrityMotorController(object):
    """A class to control a single motor on the tensegrity."""
    WRITE_HANDLE = '0x14'
    READ_HANDLE = '0x11'

    def __init__(self, btAddr):
        super(TensegrityMotorController, self).__init__()
        self.btAddr = btAddr
        self.runSpeed = 0

    def start_experiment(self, value=30, verbose=True):
        """
        Start the program by sending an initial signal.

        The initial signal gives the RFDuino code a value to run for before it
        quits. This allows us to vary the experiment values easily.
        """
        self.write_data(value)
        report = self.read_data()
        while len(report) >= 1 and report[0] != value:
            if verbose:
                print("Waiting on start confirm. {}".format(report))
            self.write_data(value)
            report = self.read_data()
            sleep(0.5)
        if verbose:
            print(report)

    def check_experiment_over(self):
        """
        Make sure the current experiment has ended.

        The strut will signal 0xffff when its experiment is over. Note that this
        is a BLOCKING function. It will not return until the strut signals the
        experiment is really over.
        """
        response = self.read_data()
        resp_data = response[:2]
        while resp_data != EXP_FIN_SIG and response not in RESTART_SIGNALS:
            # print("Watiting for experiment over on {}: {}/{}".format(self.btAddr, resp_data, response))
            sleep(1)
            response = self.read_data()
            resp_data = response[:2]

    def write_data(self, data):
        """Send data to the RFDuino using gatttool."""
        data = hex_pad(data)
        # print("Writing data {}".format(data))
        gatttoolArgs = ['gatttool',
                        '--device={}'.format(self.btAddr),
                        '--char-write-req',
                        '--handle={}'.format(TensegrityMotorController.WRITE_HANDLE),
                        '--value={}'.format(data),
                        '--addr-type=random',
                        '>>',
                        'dump.txt'
                        ]
        try:
            result = call(gatttoolArgs)
        except CalledProcessError as e:
            print(e)
            return "ERROR"
        return result

    def read_data(self):
        """Read data sent from the RFDuino using gatttool."""
        gatttoolArgs = ['gatttool',
                        '--device={}'.format(self.btAddr),
                        '--char-read',
                        '-a',
                        str(TensegrityMotorController.READ_HANDLE),
                        '--addr-type=random'
                        ]
        try:
            output = check_output(gatttoolArgs).strip()
        except CalledProcessError as e:
            print(e)
            return "ERROR"
        output = output[33:]  # 33 is the number of characters prior to the response
        output = output.split(' ')
        output = [int(o, 16) for o in output if o != '']
        return output

    def run_motor(self, speed):
        """Run the motor at the given speed."""
        assert 0 <= speed <= 255, "Speed should be between 0 and 255, inclusive"
        self.runSpeed = speed

        writeFail = self.write_data(speed)
        response = self.read_data()
        while response[0] != speed:
            print(response)
            response = self.read_data()
            sleep(1)


class Tensegrity(object):
    """A class to control an entire tensegrity."""

    def __init__(self, btAddrList=[], method=None, tracking_preset=False, camera=1):
        assert type(btAddrList) == list, "Bluetooth address list must be type list"
        super(Tensegrity, self).__init__()
        self.motors = [TensegrityMotorController(btAddr) for btAddr in btAddrList]
        self.motorNum = len(self.motors)
        if method:
            self.tracker = TensTracker(camNum=camera, method=method, preset=tracking_preset)
        else:
            self.tracker = TensTracker(camNum=camera)
        self.startup()

    def __len__(self):
        return self.motorNum

    def __contains__(self, other):
        return other in self.motors

    def __getitem__(self, i):
        return self.motors[i]

    @property
    def tens_pos(self):
        return self.tracker.tens_position

    def add_motor(self, motor):
        self.motors.append(motor)
        self.motorNum += 1

    def startup(self):
        """Check the startup codes when the Tens is stated."""
        for motorNum in range(self.motorNum):
            motor = self.motors[motorNum]
            report = motor.read_data()
            if report == [255, 255, 0, 0]:
                return
            while len(report) != 3:
                print(report)
                report = motor.read_data()
                sleep(1)
            print("Motor {} Start...".format(motorNum))
            print("SD Card Shield: {}".format(ERROR_CODES[report[0]]))
            print("Data File Open: {}".format(ERROR_CODES[report[1]]))
            print("Accelerometer Start: {}".format(ERROR_CODES[report[2]]))

    def run_experiment(self, value_set=[30], verbose=False, exp_time=10):
        """Start a single experiment."""
        for motorNum in range(self.motorNum):
            self.motors[motorNum].start_experiment(value_set[motorNum], verbose)
        sleep(exp_time)

    def check_experiment_over(self):
        """
        Make sure the current experiment has ended.

        Each strut will signal 0xffff when its experiment is over. We want to
        ensure each strut is sending this signal to ensure the experiment is over.
        """
        for motorNum in range(self.motorNum):
            self.motors[motorNum].check_experiment_over()

    def run_freq_set(self, freq_list):
        """
        Run the tensegrity's motors at the given frequencies.

        The length of the frequency list should be equal to the number of motors
        on the tensegrity. Each value in the list should be an int between 0 and
        255, inclusive. Each value in the list is assigned to the motor in the
        same index as the frequency value.
        """
        assert type(freq_list) == tuple, "Frequency list should be type list"
        assert len(freq_list) == self.motorNum, "# of frequencies should equal # of motors"
        assert all([type(val) == int for val in freq_list]), "All frequency values should be ints"
        assert all([0 <= val <= 255 for val in freq_list]), "All frequency values should be in [0,255]"
        for motor in range(self.motorNum):
            self.motors[motor].start_experiment(value=freq_list[motor], verbose=False)
        sleep(30)

    def stop(self):
        """Stop all motion."""
        self.run_freq_set([0, ] * self.motorNum)


def hex_pad(data):
    data = hex(data)
    if len(data) == 3: data = data[:2] + '0' + data[2]
    return data


def create_tens(nums, method=None):
    struts = []
    if 0 in nums:
        struts.append('CA:A3:11:A7:81:FD')
    if 1 in nums:
        struts.append('E7:50:27:0F:4A:CB')
    if 2 in nums:
        struts.append('EE:96:30:9E:CD:9D')
    if 3 in nums:
        struts.append('DD:4C:C3:17:D9:21')
    if 5 in nums:
        struts.append('E2:22:B3:44:73:F0')
    if 6 in nums:
        struts.append('D5:0D:E4:1F:30:8F')
    if 7 in nums:
        struts.append('C2:72:E6:99:E3:9E')
    if 8 in nums:
        struts.append('E1:3F:AE:94:34:AF')
    if 9 in nums:
        struts.append('E2:D8:73:53:F4:34')
    VVValtr = Tensegrity(struts, method=method)  # , valtr1, valtr2])
    return VVValtr


if __name__ == '__main__':
    v = create_tens([5, 6, 9], 2)

    finished=False
    while not finished:
        freqs = []
        for i in range(len(v)):
            freqs.append(input("Frequency for motor {} >> ".format(i)))

        startPos = v.tracker.tens_position
        print("Starting at {}".format(startPos))

        v.run_experiment(value_set=freqs, verbose=True)
        sleep(15)

        endPos = v.tracker.tens_position
        print("Ending at {}".format(endPos))

        dist = Point(startPos.x - endPos.x,
                     startPos.y - endPos.y)
        print("Distance: {}".format(dist))
