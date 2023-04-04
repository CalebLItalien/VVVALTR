from random import randint
from datetime import datetime
from time import sleep
from collections import namedtuple
from math import sqrt
import csv
import os

import numpy as np

import numpy as np

from pyGPGO.covfunc import matern32
from pyGPGO.acquisition import Acquisition
from pyGPGO.surrogates.GaussianProcess import GaussianProcess
from pyGPGO.GPGO import GPGO

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from runTens import Tensegrity
from tensTracking import Point, WHITE, SUB

VALTR0 = 'CA:A3:11:A7:81:FD'
VALTR1 = 'E7:50:27:0F:4A:CB'
VALTR2 = 'EE:96:30:9E:CD:9D'
VALTR3 = 'DD:4C:C3:17:D9:21'

# Brushless Motors
VALTR5 = 'E2:22:B3:44:73:F0'
VALTR7 = 'D8:DE:C8:4C:AF:92'
VALTR8 = 'C2:72:E6:99:E3:9E'

MAX_SPEED = 0xff  # highest safe speed for current motors
MIN_SPEED = 0x17  # 0x17 is first speed motor spins at, so this is essentially 0

# Index meanings for data lists
FREQS = 0
DIST = 1
START = 2
END = 3

VVVALTR_DIR = "/".join(os.path.realpath(__file__).split('/')[0:-1])


class LearningMethod(object):
    """Generic class for using machine learning to generate gaits."""
    def __init__(self, btAddrList, testNum=5, testTime=30, tracking_method=WHITE):
        assert len(btAddrList) > 0, "At least one Bluetooth address is needed"
        self.tens = Tensegrity(btAddrList, method=tracking_method)
        self.tracker = self.tens.tracker
        self.testNum = testNum
        self.results = []
        self.tested = []
        self.bestResult = [(MIN_SPEED,) * len(self.tens), 0, 0, 0]
        self.testTime = testTime
        self.dataFileName = VVVALTR_DIR + "/data/test-{}".format(datetime.now())

    def save_results(self):
        with open(self.dataFileName, 'w') as dataFile:
            dataWriter = csv.writer(dataFile)
            for res in self.results:
                dataWriter.writerow(res)

    def load_results(self, filename):
        with open(filename, 'r') as dataFile:
            dataReader = csv.reader(dataFile)
            for row in dataReader:
                dist = row[DIST]
                if dist > self.bestResult[DIST]:
                    self.bestResult = row
                self.results.append(row)


class RandomHillClimber(LearningMethod):
    """A class which uses a stochastic hill climber to generate gaits."""
    def __init__(self, btAddrList, testNum=5):
        LearningMethod.__init__(self, btAddrList, testNum)
        self.currBest = [0] * len(self.tens)
        self.dataFileName = VVVALTR_DIR + "/data/RHC-test-{}.csv".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        print("Data file is {}".format(self.dataFileName))

    def __generate_freqs(self):
        """Generate a set of n frequencies for the n motors."""
        freqList = [MIN_SPEED,] * len(self.tens)
        while tuple(freqList) in self.tested:
            freqList = []
            for x in range(len(self.tens)):
                freqList.append(randint(MIN_SPEED, MAX_SPEED))

        return tuple(freqList)

    def run_experiment(self, time=30):
        """Generate freqs, run them for test time, the return results."""
        testFreqs = self.__generate_freqs()
        startPos = self.tracker.tens_position

        print("Testing frequencies: {}".format(testFreqs))
        self.tens.run_experiment(verbose=True)
        self.tens.run_freq_set(testFreqs)
	endPos = self.tracker.tens_position

	sleep(10)  # offset to ensure motor is done moving

        dist = sqrt((endPos.x - startPos.x)**2 + (endPos.y - startPos.y)**2)

        result = [testFreqs, dist, startPos, endPos]

        self.results.append(result)
        self.tested.append(testFreqs)
        print("Distance: {}".format(dist))

        if dist > self.bestResult[DIST]:
            self.bestResult = result
            print("New best found!")
        return result

    def verify_completion(self):
        """Checks each strut to ensure they finished the experiment."""
        for strut in self.tens.motors:
            response = strut.read_data()[:2]
            while response != [255, 255]:
                print("Motor {} verification: {}".format(strut.btAddr, response))
                response = strut.read_data()[:2]
                sleep(0.5)

    def hill_climb(self, iterations=100):
        """Run a hill climber for n iterations."""
        for i in range(iterations):
            self.run_experiment()
            sleep(5)
            self.verify_completion()
        print("Best result: {}".format(self.bestResult))

class BayesianOptimizer(LearningMethod):
    def __init__(self, btAddrList, testNum=5, *args, **kwargs):
        LearningMethod.__init__(self, btAddrList, testNum, *args, **kwargs)

        self.currBest = [0] * len(self.tens)

        self.dataFileName = VVVALTR_DIR + "/data/BO-test-{}.csv".format(datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        print("Data file is {}".format(self.dataFileName))

    def experiment(self, *testFreqs):
        testFreqs = tuple(testFreqs)
        print testFreqs
        startPos = self.tracker.tens_position

        print("Testing frequencies: {}".format(testFreqs))
        self.tens.run_freq_set(testFreqs)
        
        endPos = self.tracker.tens_position

	sleep(10)

        dist = sqrt((endPos.x - startPos.x)**2. + (endPos.y - startPos.y)**2.)\

        result = [testFreqs, dist, startPos, endPos]
        self.results.append(result)
        self.tested.append(testFreqs)
        print("Distance: {}".format(dist))

        return dist

    def run_optimizer(self, iterations=40):
        cov = matern32()
        gp = GaussianProcess(cov)
        acq = Acquisition(mode='ExpectedImprovement')

        param = {
            'x': ('int', [MIN_SPEED, MAX_SPEED]),
            'y': ('int', [MIN_SPEED, MAX_SPEED])
        }

        meta = lambda x, y: self.experiment(int(x), int(y))
	
	#self.tens.start_experiment(verbose=False)
        gpgo = GPGO(gp, acq, meta, param)
        gpgo.run(max_iter=iterations)

def main(method):
    # RHC = RandomHillClimber([VALTR0, VALTR1, VALTR2])
    # RHC.hill_climb(iterations=10)
    # RHC.save_results()

    BO = BayesianOptimizer([VALTR5, VALTR7], tracking_method=method)
    BO.run_optimizer(iterations=40)
    BO.save_results()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        method_raw = sys.argv[1]
    else:
        method_raw = 'white'
    method = WHITE if method_raw == 'white' else SUB
    main(method)
