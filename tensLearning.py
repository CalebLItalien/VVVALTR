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
from pickle import load, dump

import numpy as np

import numpy as np

from pyGPGO.covfunc import matern32
from pyGPGO.acquisition import Acquisition
from pyGPGO.surrogates.GaussianProcess import GaussianProcess
from pyGPGO.GPGO import GPGO

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from runTens import Tensegrity
from tensTracking import WHITE, SUB, PUFFS

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
        self.tens = Tensegrity(btAddrList, method=tracking_method, tracking_preset=tracking_preset)
        self.tracker = self.tens.tracker
        self.testNum = testNum
        self.results = []
        self.tested = []
        self.bestResult = [(MIN_SPEED,) * len(self.tens), 0]
        self.testTime = testTime
        self.dataFileName = VVVALTR_DIR + "/data/test-{}".format(datetime.now())
        self.tens_vers = tens_vers
        self.code_vers = code_vers
        # self.start_results(['Freqs', 'Disp', 'Start', 'End', 'Tens', 'Code'])

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
            sleep(2)
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
        self.mode = 'UCB'
        self.mode_short = 'ucb05'
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
        while dist_to_frame_center > 115:
            print("Center tensegrity, dist to center is ", dist_to_frame_center)
            sleep(2)
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
        acq = Acquisition(mode=self.mode, beta=0.5)

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
        # with open(self.fileName + ".backup", 'w') as backup:
        #     dump(self.gpgo, backup)


def main(optimizer=None):
    # RHC = RandomHillClimber([VALTR5, VALTR6, VALTR7])
    # RHC.hill_climb(iterations=61)
    # RHC.save_results()

    BO = BayesianOptimizer([VALTR5, VALTR6, VALTR9])
    BO.run_optimizer(iterations=60)
    # # BO.save_results()


if __name__ == '__main__':
    import sys

    # if len(sys.argv) > 1:
    #     load_file = sys.argv[1]
    #     with open(load_file, 'r') as prev_file:
    #         optimizer = load(prev_file)
    #     main(optimizer)
    main()
