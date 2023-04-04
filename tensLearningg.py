"""
mcvickec 10/21 - Fails looking for USB camera
"""
from random import randint
from datetime import datetime
from time import sleep
from collections import namedtuple
from math import sqrt
import csv
import os
import asyncio
import xml.etree.ElementTree as ET
import pkg_resources

import qtm

import math
import numpy as np
import time
from pickle import load, dump

import numpy as np

import numpy as np
import random

from tracker_class import QtmTracker

from runTens import Tensegrity

from tensTracking import WHITE, SUB, PUFFS
import threading
import time



VALTR0 = 'CA:A3:11:A7:81:FD'
VALTR1 = 'E7:50:27:0F:4A:CB'
VALTR2 = 'EE:96:30:9E:CD:9D'
VALTR3 = 'DD:4C:C3:17:D9:21'

# Brushless Motors
VALTR5 = 'E2:22:B3:44:73:F0'
VALTR6 = 'D5:0D:E4:1F:30:8F'
VALTR7 = 'C2:72:E6:99:E3:9E'
VALTR8 = 'E1:3F:AE:94:34:AF'
VALTR9 = 'E2:D8:73:53:F4:34'

MAX_SPEED = 0xff  # highest safe speed for current motors
MIN_SPEED = 0  # 0x17 is first speed motor spins at, so this is essentially 0

# Index meanings for data lists
FREQS = 1
DIST = 2
START = 3
END = 4
TENS_VERS = 5
CODE_VERS = 6
MIN_VAL = 7  # indicates minimum speed test considered, see MIN_SPEED
MAX_VAL = 8  # indicates minimum speed test considered, see MIN_SPEED
# There may be extra data points, depending on the learning strategy

VVVALTR_DIR = "/".join(os.path.realpath(__file__).split('/')[0:-1])


class LearningMethod(object):
    """Generic class for using machine learning to generate gaits."""

    def __init__(self, btAddrList,
                 testNum=5, testTime=30,
                 tracking_method=SUB, tracking_preset = False,
                 tens_vers=1.0, code_vers=1.0):
        assert len(btAddrList) > 0, "At least one Bluetooth address is needed"
        self.tens = Tensegrity(btAddrList, method=tracking_method)
        self.testNum = testNum
        self.results = []
        self.tested = []
        self.bestResult = [(MIN_SPEED,) * len(self.tens), 0]
        self.testTime = testTime
        self.dataFileName = VVVALTR_DIR + "/data/test-{}".format(datetime.now())
        self.tens_vers = tens_vers
        self.code_vers = code_vers

    def start_results(self, field_names):
        with open(self.dataFileName, 'w') as data_file:
            dataWriter = csv.writer(data_file)
            dataWriter.writerow(field_names)

    def save_results(self):
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            for res in self.results:
                new_row = res + [self.tens_vers, self.code_vers]
                dataWriter.writerow(res)

    def load_results(self, filename):
        with open(filename, 'r') as dataFile:
            dataReader = csv.reader(dataFile)
            for row in dataReader:
                dist = row[DIST]
                if dist > self.bestResult[DIST]:
                    self.bestResult = row
                self.results.append(row)

    def save_single_result(self, result):
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            dataWriter.writerow(result)


class RandomHillClimber(LearningMethod):
    """A class which uses a stochastic hill climber to generate gaits."""

    def __init__(self, btAddrList, testNum=5):
        LearningMethod.__init__(self, btAddrList, testNum)
        self.currBest = 0.
        self.dataFileName = VVVALTR_DIR + "/data/RHC-test-{}.csv".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        print("Data file is {}".format(self.dataFileName))
        self.iter_num = 0

    def __generate_freqs(self):
        """Generate a set of n frequencies for the n motors."""
        freqList = [randint(0, 255) for i in range(len(self.tens))]
        while tuple(freqList) in self.tested:
            freqList = []
            for x in range(len(self.tens)):
                freqList.append(randint(MIN_SPEED, MAX_SPEED))

        return tuple(freqList)

    def run_experiment(self, time=30):
        """Generate freqs, run them for test time, the return results."""
        self.iter_num += 1
        testFreqs = self.__generate_freqs()
        print("{}: {}".format(self.iter_num, testFreqs))
        startPos = self.tracker.tens_position

        self.tens.run_experiment(testFreqs, verbose=False)
        self.tens.check_experiment_over()
        # self.tens.run_freq_set(testFreqs)

        endPos = self.tracker.tens_position
        dist = sqrt((endPos.x - startPos.x) ** 2 + (endPos.y - startPos.y) ** 2)
        print("{}: {} -> {}".format(self.iter_num, testFreqs, dist))
        self.currBest = dist if dist > self.currBest else self.currBest
        result = [self.iter_num, testFreqs, dist, startPos, endPos, self.tens_vers, self.code_vers, MIN_SPEED,
                  MAX_SPEED, 'Rand', 'Rand', self.currBest]
        self.results.append(result)
        self.tested.append(testFreqs)

        f_center_x, f_center_y = self.tracker.frame_center
        dist_to_frame_center = sqrt((endPos.x - f_center_x) ** 2 + (endPos.y - f_center_y) ** 2)
        while dist_to_frame_center > 120:
            print("Center tensegrity, dist to center is ", dist_to_frame_center)
            sleep(2.5)
            cur_pos = self.tracker.tens_position
            dist_to_frame_center = sqrt((cur_pos.x - f_center_x) ** 2 + (cur_pos.y - f_center_y) ** 2)
        sleep(2)  # let tensegrity rest

        self.save_single_result(result)
        return dist

    def hill_climb(self, iterations=100):
        """Run a hill climber for n iterations."""
        for i in range(iterations):
            self.run_experiment()
        print("Best result: {}".format(self.currBest))


class BayesianOptimizer(LearningMethod):
    def __init__(self, btAddrList, testNum=5, optimizer=None, *args, **kwargs):
        LearningMethod.__init__(self, btAddrList, testNum, *args, **kwargs)
        self.mode = 'ExpectedImprovement'
        self.mode_short = 'A4'
        self.iter_num = 0
        self.gpgo = optimizer

        self.currBest = 0

        self.fileName = VVVALTR_DIR + "/data/BO-test-{}".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.dataFileName = self.fileName + ".csv"
        print("Data file is {}".format(self.dataFileName))

    def experiment(self, testFreqs):
        self.iter_num += 1
        testFreqs = tuple(testFreqs)
        # print("Testing frequencies: {}".format(testFreqs))
        startPos = self.tracker.tens_position

        self.tens.run_experiment(testFreqs, verbose=False)
        self.tens.check_experiment_over()
        # self.tens.run_freq_set(testFreqs)

        endPos = self.tracker.tens_position
        dist = sqrt((endPos.x - startPos.x) ** 2 + (endPos.y - startPos.y) ** 2)
        self.currBest = dist if dist > self.currBest else self.currBest
        result = [self.iter_num, testFreqs, dist, startPos, endPos, self.tens_vers, self.code_vers, MIN_SPEED, MAX_SPEED]
        self.results.append(result)
        self.tested.append(testFreqs)

        f_center_x, f_center_y = self.tracker.frame_center
        dist_to_frame_center = sqrt((endPos.x - f_center_x) ** 2 + (endPos.y - f_center_y) ** 2)
        while dist_to_frame_center > 120:
            print("Center tensegrity, dist to center is ", dist_to_frame_center)
            sleep(2.5)
            cur_pos = self.tracker.tens_position
            dist_to_frame_center = sqrt((cur_pos.x - f_center_x) ** 2 + (cur_pos.y - f_center_y) ** 2)
        sleep(2)  # let tensegrity rest

        self.save_single_result(result)
        return dist

    def run_optimizer(self, iterations=40):
        if self.gpgo is not None:
            self.gpgo.run(max_iter=iterations)
            return
        cov = matern32()
        gp = GaussianProcess(cov)
        gp = self.fit_priors(gp)
        acq = Acquisition(mode=self.mode)#, beta=2.0)

        param = {}

        for m_num in range(len(self.tens)):
            param[str(m_num)] = ('int', [MIN_SPEED, MAX_SPEED])

        meta = lambda **kwargs: self.experiment([int(kwargs[a]) for a in kwargs])

        self.gpgo = GPGO(gp, acq, meta, param)
        self.gpgo.run(max_iter=iterations)

    def fit_priors(self, gp):
        """
        Fits a set of priors to the Gaussian Process used as a surrogate function.

        This allows us to use Bayesian optimization with prior knowledge, which has been proven helpful.
        :param gp: The Gaussian Process to update
        :return: The fitted Gaussian Process
        """
        p_samples = np.array([[0, 0, 0],
                              [255, 255, 255],
                              [255, 255, 100],
                              [255, 100, 255],
                              [100, 255, 255]])
        p_results = np.array([0., 50., 75., 75., 75.])
        gp.fit(p_samples, p_results)
        return gp

    def save_single_result(self, result):
        result += ['BayesOpt', self.mode_short, self.currBest]
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            dataWriter.writerow(result)

class RandomOptimizer(LearningMethod):
    def __init__(self, btAddrList, testNum=5, optimizer=None, *args, **kwargs):
        LearningMethod.__init__(self, btAddrList, testNum, *args, **kwargs)
        self.mode_short = 'A2b20'
        self.iter_num = 0
        self.currBest = 0

        self.fileName = VVVALTR_DIR + "/data/Random-test-{}".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.dataFileName = self.fileName + ".csv"
        print("Data file is {}".format(self.dataFileName))

    def __generate_freqs(self):
        """Generate a set of n frequencies for the n motors."""
        freqList = [randint(0, 255) for i in range(len(self.tens))]
        while tuple(freqList) in self.tested:
            freqList = []
            for x in range(len(self.tens)):
                freqList.append(randint(MIN_SPEED, MAX_SPEED))

        return tuple(freqList)

    def experiment(self, testFreqs):
        self.iter_num += 1
        testFreqs = tuple(testFreqs)
        startPos = self.tracker.tens_position

        self.tens.run_experiment(testFreqs, verbose=False)
        self.tens.check_experiment_over()

        endPos = self.tracker.tens_position
        dist = sqrt((endPos.x - startPos.x) ** 2 + (endPos.y - startPos.y) ** 2)
        self.currBest = dist if dist > self.currBest else self.currBest
        result = [self.iter_num, testFreqs, dist, startPos, endPos, self.tens_vers, self.code_vers, MIN_SPEED, MAX_SPEED]
        self.results.append(result)
        self.tested.append(testFreqs)

        f_center_x, f_center_y = self.tracker.frame_center
        dist_to_frame_center = sqrt((endPos.x - f_center_x) ** 2 + (endPos.y - f_center_y) ** 2)
        while dist_to_frame_center > 120:
            print("Center tensegrity, dist to center is ", dist_to_frame_center)
            sleep(2.5)
            cur_pos = self.tracker.tens_position
            dist_to_frame_center = sqrt((cur_pos.x - f_center_x) ** 2 + (cur_pos.y - f_center_y) ** 2)
        sleep(2)  # let tensegrity rest

        self.save_single_result(result)
        return dist

    def run_optimizer(self, iterations=60):
        for i in range(iterations):
            freqs = self.__generate_freqs()
            old_best = self.currBest
            dist = self.experiment(freqs)
            new_best = self.currBest

            if new_best != old_best:
                print(str(i+1), "  ", freqs, "new best: ", new_best,"   ", new_best)
            else:
                print(str(i+1), "  ", freqs, "       ", dist,"   ", new_best)


    def save_single_result(self, result):
        result += ['RandomOpt', self.mode_short, self.currBest]
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            dataWriter.writerow(result)

class BestValues(LearningMethod):
    def __init__(self, btAddrList, testNum=5, optimizer=None, *args, **kwargs):
        LearningMethod.__init__(self, btAddrList, testNum, *args, **kwargs)
        self.mode_short = 'bvt'
        self.iter_num = 0
        self.currBest = 0

        self.fileName = VVVALTR_DIR + "/data/Best-test-{}".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.dataFileName = self.fileName + ".csv"
        print("Data file is {}".format(self.dataFileName))

    def experiment(self, testFreqs):
        self.iter_num += 1
        testFreqs = tuple(testFreqs)
        print("Testing frequencies: {}".format(testFreqs))

        self.tens.run_experiment(testFreqs, verbose=False)
        self.tens.check_experiment_over()

        '''
        f_center_x, f_center_y = self.tracker.frame_center
        dist_to_frame_center = sqrt((endPos.x - f_center_x) ** 2 + (endPos.y - f_center_y) ** 2)
        while dist_to_frame_center > 120:
            print("Center tensegrity, dist to center is ", dist_to_frame_center)
            sleep(2.5)
            cur_pos = self.tracker.tens_position
            dist_to_frame_center = sqrt((cur_pos.x - f_center_x) ** 2 + (cur_pos.y - f_center_y) ** 2)
        sleep(2)  # let tensegrity rest
        '''
        '''
        print(result)
        self.save_single_result(result)
        '''
        return 1

    def save_single_result(self, result):
        result += ['Best', self.mode_short, self.currBest]
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            dataWriter.writerow(result)

class MapElites(LearningMethod):

    def __init__(self, btAddrList, testNum=5, optimizer=None, *args, **kwargs):
        LearningMethod.__init__(self, btAddrList, testNum, *args, **kwargs)
        self.mode_short = 'mapelites'
        self.iter_num = 0
        self.currBest = 0

        self.fileName = VVVALTR_DIR + "/data/MAP-test-{}".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.dataFileName = self.fileName + ".csv"
        print("Data file is {}".format(self.dataFileName))

        self.tracker = QtmTracker("10.76.30.91")
        self.points = []
        self.Freqs = [0, 0, 0]
        self.time = 30

    def poses(self):
        self.points = []
        interval = 5
        iters = int(self.time/interval)
        for i in range(iters + 1):
            self.points.append(self.tracker.get_current_pos())
            print(i)
            time.sleep(interval)

    def control(self):
        self.tens.run_experiment(self.Freqs, verbose=False)

    def experiment(self, testFreqs):
        self.iter_num += 1
        testFreqs = list(testFreqs)
        self.Freqs = testFreqs

        if len(self.tested) >= 80:
            oldFreq = random.choice(self.tested) #random selection

            newFreq = []
            for i in oldFreq:
                temp = i + random.randint(-20,20)
                if temp > 253:
                    temp = 253
                elif temp < 0:
                    temp = 0

                newFreq.append(temp)
            testFreqs = newFreq
            self.Freqs = newFreq               #random variation
        print("Testing frequencies: {}".format(testFreqs))

        threading.Thread(target=self.poses).start()
        threading.Thread(target=self.control).start()

        self.tens.check_experiment_over()
        print(self.points)

        results = []
        results.append((0, 0))
        for i in range(len(self.points) - 1):
            trans = self.tracker.get_translation(self.points[0][0], self.points[i+1][0])
            rot = self.tracker.get_rotation(self.points[0][1], self.points[i+1][1])
            x = self.points[i+1][0][0] - self.points[0][0][0]
            y = self.points[i+1][0][1] - self.points[0][0][1]
            results.append((trans, rot,(x, y)))

        if len(results) == 1: # if didn't write sucessfully the first time
            self.poses()
            for i in range(len(self.points) - 1):
                trans = self.tracker.get_translation(self.points[0][0], self.points[i+1][0])
                rot = self.tracker.get_rotation(self.points[0][1], self.points[i+1][1])
                x = self.points[i+1][0][0] - self.points[0][0][0]
                y = self.points[i+1][0][1] - self.points[0][0][1]
                results.append((trans, rot,(x, y)))


        print(results)

        '''
        endPos = self.tracker.get_current_pos()[0]
        endAng = self.tracker.get_current_pos()[1]
        dist = self.tracker.get_translation(startPos, endPos)
        rotation = self.tracker.get_rotation(startAng, endAng)
        '''
        #dist = 0
        '''
        self.currBest = dist if dist > self.currBest else self.currBest
        '''
        result = [self.iter_num, testFreqs] + results
        self.results.append(result)
        self.tested.append(testFreqs)

        sleep(4)
        if self.iter_num%6 == 0:
            centered = False
            while not centered:
                c_str = input("Please center the tensegrity! y/n: ")
                if c_str == "y":
                    centered = True
                    self.tracker = QtmTracker("10.76.30.91")

        self.save_single_result(result)
        dist = 0
        return dist

    def save_single_result(self, result):
        # result += ['ww', self.mode_short, self.currBest]
        with open(self.dataFileName, 'a') as dataFile:
            dataWriter = csv.writer(dataFile)
            dataWriter.writerow(result)

    def run_optimizer(self, iterations=60):
        for i in range(iterations):
            freqs = self.__generate_freqs()
            print(str(i+1), "  ", freqs)
            self.experiment(freqs)

    def __generate_freqs(self):
        """Generate a set of n frequencies for the n motors."""
        freqList = [randint(0, 255) for i in range(len(self.tens))]
        while tuple(freqList) in self.tested:
            freqList = []
            for x in range(len(self.tens)):
                freqList.append(randint(MIN_SPEED, MAX_SPEED))
        return tuple(freqList)

def main(optimizer=None):


    MP = MapElites([VALTR5, VALTR6, VALTR9])
    MP.run_optimizer(iterations=180)

if __name__ == '__main__':
    main()
