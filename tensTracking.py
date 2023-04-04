from time import sleep
import math
from math import asin
import cv2
from threading import Thread
import numpy as np
from copy import copy
from collections import namedtuple

Point = namedtuple('Point', ['x', 'y'])

BLUR_SIZE = 7
PUFF_BLUR_SIZE = 5

PIX_PER_CM = 3.5

WHITE = 1
SUB = 2
PUFFS = 3

PRESET_PUFFS = [(np.array([0, 120, 0], dtype=np.uint8), np.array([9, 255, 255], dtype=np.uint8)),
                (np.array([100, 120, 0], dtype=np.uint8), np.array([109, 255, 255], dtype=np.uint8)),
                (np.array([160, 120, 0], dtype=np.uint8), np.array([170, 255, 255], dtype=np.uint8)),
                ]

TEST_AREA = {'ul': (0, 0),
             'br': (640, 480)}


class TensTracker(object):

    def __init__(self, camNum=None, display=True, method=SUB, preset=False, record=False):
        # Stuff to get the camera running
        self.__raw_frame = None
        self.__draw_frame = None
        self.__mask_frames = [None, None, None]
        self.__find_camera(camNum)
        self.__frame_updater = Thread(target=self.__update_frame, args=())
        self.__frame_updater.daemon = True
        self.__frame_updater.start()

        self.test_area = {'ul': (0, 0),
                          'br': (640, 480)}


        while self.__raw_frame is None:
            continue

        self.no_tens_warned = False

        if method == SUB:
            self.get_base_img()
            print("Base image obtained. Place tensegrity")
            sleep(10)

        if method == PUFFS:
            self.puffs = []
            self.tens_angle = 0.0
            if not preset:
                self.__get_puffs()
            else:
                self.puffs = PRESET_PUFFS

        self.get_test_area()

        if record:
            fourcc = cv2.cv.FOURCC(*'XVID')
            self.record = True
            self.vid_record = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

        self.tensX = -1
        self.tensY = -1
        self.method = method

        self.__pos_updater = Thread(target=self.__update_pos, args=())
        self.__pos_updater.daemon = True
        self.__pos_updater.start()

        self.__frame_displayer = Thread(target=self.display_frame, args=())
        self.__frame_displayer.daemon = True
        if display:
            self.__frame_displayer.start()
        cv2.destroyAllWindows()


        # self.puffs = []
        # self.__get_puffs()  # names I never thought I'd give a method for $1000

    @property
    def frame_center(self):
        f_shape = self.get_frame().shape
        y_center = int(f_shape[0]/2)
        x_center = int(f_shape[1]/2)
        return x_center, y_center

    def get_raw_frame(self):
        """
        Get the current raw frame (undrawn on) from the camera.

        This function DOES NOT GUARANTEE A RESULT. The function is made to be
        non-blocking, so it doesn't ensure it returns an image.

        :return: A 3d numpy array with colors as BGR or None
        """
        if self.__raw_frame != [] and self.__raw_frame is not None:
            return np.copy(self.__raw_frame)
        else:
            return None

    def get_frame(self):
        """
        Get the current frame from the camera

        This function DOES NOT GUARANTEE A RESULT. The function is made to be
        non-blocking, so it doesn't ensure it returns an image.

        :return: A 3d numpy array with colors as BGR
        """
        if self.__draw_frame != [] and self.__draw_frame is not None:
            return np.copy(self.__draw_frame)
        else:
            return None

    def get_base_img(self):
        img = self.get_raw_frame()
        while img is None:
            img = self.get_raw_frame()
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        preBlur = cv2.blur(gray, (BLUR_SIZE, BLUR_SIZE))
        blur = cv2.medianBlur(preBlur, BLUR_SIZE)
        self.baseImg = blur

    def get_test_area(self):
        frame = self.get_raw_frame()
        while frame is None:
            frame = self.get_raw_frame()
        print("Select the upper left boundary puffball")
        ul_puff_bounds = self.__get_puff_bounds()

        print("Select the bottom right boundary puffball")
        br_puff_bounds = self.__get_puff_bounds()

        ul_puff_pos = self.__get_puff_center(frame, ul_puff_bounds)
        br_puff_pos = self.__get_puff_center(frame, br_puff_bounds)

        TEST_AREA['ul'] = ul_puff_pos
        TEST_AREA['br'] = br_puff_pos

        wid = br_puff_pos[0] - ul_puff_pos[0]
        hgt = br_puff_pos[1] - ul_puff_pos[1]

        if self.baseImg is not None:
            self.baseImg = self.baseImg[ul_puff_pos[1]:br_puff_pos[1], ul_puff_pos[0]:br_puff_pos[0]]

        print("Image is {} x {}".format(wid, hgt))

    @property
    def tens_position(self):
        return Point(self.tensX, self.tensY)

    def display_frame(self, pos=False):
        while True:
            img = self.get_frame()
            if img is None:
                continue
            if pos:
                print(self.tens_position)
            cv2.imshow("Tracker View", img)

            # if self.record:
            #     self.vid_record.write(img)

            # DEBUG MODE: DISPLAY MASK FRAME
            # m_frame = cv2.bitwise_or(self.__mask_frames[0], self.__mask_frames[1])
            # m_frame = cv2.bitwise_or(m_frame, self.__mask_frames[2])
            # m_view = cv2.bitwise_and(img, img, mask=m_frame)
            # cv2.imshow("Mask View", m_view)

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break

    def __find_camera(self, camNum):
        """
        Searches through cameras to find one that works. Asks the user to
        verify that the found camera is correct before using it.
        """
        max_cams = 4
        if camNum is not None:
            try:
                self.capture = cv2.VideoCapture(camNum)
            except:
                print("Camera {} is bad".format(camNum))
            if self.capture:
                ret, frame = self.capture.read()
                cv2.imshow('Camera View', frame)
                k = cv2.waitKey(1) & 0xFF
                uc = raw_input("Is this the correct camera?")
                if uc.upper() == 'N':
                    cv2.destroyAllWindows()
                else:
                    print("Using camera {}".format(camNum))
                    cv2.destroyAllWindows()
                    return True
        for cam in range(max_cams):
            print("Testing camera {}".format(cam))
            try:
                self.capture = cv2.VideoCapture(cam)
            except:
                print("Camera {} is bad".format(cam))
            if self.capture:
                ret, frame = self.capture.read()
                cv2.imshow('Camera View', frame)
                k = cv2.waitKey(0) & 0xFF
                uc = raw_input("Is this the correct camera?")
                if uc.upper() == 'N':
                    cv2.destroyAllWindows()
                else:
                    print("Using camera {}".format(cam))
                    cv2.destroyAllWindows()
                    break
        assert self.capture is not None, "Couldn't find camera"

    def __update_frame(self):
        """
        Update the current frame instance variable by retrieving the next frame
        from the camera.

        Meant to run as a separate thread.
        """
        while True:
            ret, img = self.capture.read()
            xmax, ymax = TEST_AREA['br']
            xmin, ymin = TEST_AREA['ul']
            img = img[ymin:ymax, xmin:xmax]
            self.__raw_frame = img
            # print("__update_frame():", TEST_AREA)
            # sleep(1)

    def __update_pos(self):
        """
        Update the current tensegrity location and drawn frame.

        Meant to run as a separate thread.
        """
        while True:
            if self.method == WHITE:
                self.__draw_frame = self.__find_tens_white(copy(self.__raw_frame))
            elif self.method == SUB:
                self.__draw_frame = self.__find_tens_subtraction(copy(self.__raw_frame))
            else:
                self.__draw_frame = self.__find_tens_puff(copy(self.__raw_frame))

    def __find_tens_white(self, img):
        """
        Takes an image, finds the tensegrity, and returns a painted image.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur = cv2.blur(gray, (64, 64))
        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(copy(thresh), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            mmnts = cv2.moments(contours[0])
            cx = int(mmnts['m10']/mmnts['m00'])
            cy = int(mmnts['m01']/mmnts['m00'])
            cv2.drawContours(img, contours, 0, (0, 0xff, 0), 3)
            cv2.circle(img, (cx, cy), 3, (0, 0, 0xff), -1)
            # print(cx, cy)
            self.tensX = cx
            self.tensY = cy
            self.no_tens_warned = False
        except ZeroDivisionError:
            pass
        except IndexError:
            if not self.no_tens_warned:
                print("WARNING: No Tensegrity found!")
                self.no_tens_warned = True
                return img
        return img

    def __find_tens_subtraction(self, img):
        """
        Takes an image, finds the tensegrity, and returns a painted image.
        """
        if img.shape[:2] != self.baseImg.shape:
            # print("Size mismatch: ", img.shape, self.baseImg.shape)
            return img

        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        preBlur = cv2.blur(gray, (BLUR_SIZE, BLUR_SIZE))
        blur = cv2.medianBlur(preBlur, BLUR_SIZE)

        subImg = self.baseImg - blur
        _, subThreshA = cv2.threshold(subImg, 200, 0, cv2.THRESH_TOZERO_INV)
        _, subThresh = cv2.threshold(subThreshA, 25, 255, cv2.THRESH_BINARY)

        cv2.imwrite('base_img.png', self.baseImg)
        cv2.imwrite('gray_raw.png', gray)
        cv2.imwrite('sub_raw.png', subImg)
        cv2.imwrite('sub_threshA.png', subThreshA)
        cv2.imwrite('sub_thresh.png', subThresh)

        contours, _ = cv2.findContours(copy(subThresh), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        biggest_cont = list(sorted(contours, key=lambda x: x.size))[-1]
        try:
            mmnts = cv2.moments(biggest_cont)
            cx = int(mmnts['m10']/mmnts['m00'])
            cy = int(mmnts['m01']/mmnts['m00'])
            cv2.drawContours(img, contours, -1, (0, 0xff, 0), 3)
            cv2.circle(img, (cx, cy), 3, (0, 0, 0xff), -1)
            # print(cx, cy)
            self.tensX = cx
            self.tensY = cy
            self.no_tens_warned = False
        except ZeroDivisionError:
            pass
        except IndexError:
            if not self.no_tens_warned:
                print("WARNING: No Tensegrity found!")
                self.no_tens_warned = True
                return img

        # Draw circle of radius 150 (centering boundary
        cv2.circle(img, (self.frame_center[0], self.frame_center[1]), 115, (0xff, 0, 0), 2)
        cv2.imwrite('final_view.png', img)

        return img

    def __find_tens_puff(self, img):
        if img is None:
            raise ValueError("Blob tracking error: img is None")
        try:
            puff_centers = self.__get_puff_centers(img)
        except ZeroDivisionError as e:
            print("Blob tracking error: Moment m00 is 0")
            return img
        tens_center = self.__get_tens_center(puff_centers)
        self.tensX = tens_center.x
        self.tensY = tens_center.y
        self.no_tens_warned = False
        # self.tens_angle = self.__get_tens_angle(Point(x=self.tensX, y=self.tensY), puff_centers[0])

        up_left = (tens_center.x - 15, tens_center.y - 15)
        bot_right = (tens_center.x + 15, tens_center.y + 15)
        cv2.rectangle(img, up_left, bot_right, [126, 255, 255], 2)
        cv2.circle(img, (self.tensX, self.tensY), 3, [255, 255, 255])
        cv2.line(img,
                 (tens_center.x, tens_center.y),
                 (puff_centers[0].x, puff_centers[0].y),
                 (0, 255, 255),
                 3)
        return img

    def __get_tens_angle(self, tc, pc):
        """
        Draw a line from the center of the tensegrity to the top of the frame.
        Draw another line from the center of the tensegrity to the first puff
        ball (i.e. self.puffs[0]). Calculate the angle between these two lines
        and return it as an int. Use basic trig by finding the sin^-1 of the
        ratio between the hypotenuse and opposite side of the triangle formed
        :return: Int value of the tensegrity's heading
        """
        # Point above the center of the tensegrity on the top of the screen
        ref_point = Point(x=tc.x, y=0)

        bearing_slope = (tc.y - pc.y) / (tc.x - pc.x)
        # Point where the line from center to puff hits the top of screen
        bearing_point = Point(pc.y - bearing_slope * pc.x, 0)

        opp_length = ref_point.x - bearing_point.x
        hypo_length = int(math.sqrt((tc.x - bearing_point.x) ** 2 + (tc.y - bearing_point.y) ** 2))

        angle = asin(opp_length / float(hypo_length)) * (180.0 / math.pi)
        return angle

    def __get_tens_center(self, puff_centers):
        x_vals = [center.x for center in puff_centers]
        y_vals = [center.y for center in puff_centers]
        cx = sum(x_vals)/3
        cy = sum(y_vals)/3
        return Point(x=cx, y=cy)

    def __get_puff_centers(self, frame):
        """
        Find the centers of the puffs by using algorithms
        :return: a list of (x,y) tuples for each puff
        """
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        puff_centers = []
        for puff in self.puffs:
            puff_low = puff[0]  # lower bound on color
            puff_high = puff[1]  # upper bound on color

            frame = cv2.medianBlur(frame, 3)
            mask = cv2.inRange(frame, puff_low, puff_high)
            self.__mask_frames[len(puff_centers)] = mask

            conts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            max_cont = sorted(conts, key=lambda x: x.size)[-1]

            # get the moments and find the center
            mmnts = cv2.moments(max_cont)
            cx = int(mmnts['m10'] / mmnts['m00'])
            cy = int(mmnts['m01'] / mmnts['m00'])

            puff_centers.append(Point(x=cx, y=cy))

        return puff_centers

    def __get_puff_center(self, frame, puff_bounds):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        puff_low = puff_bounds[0]  # lower bound on color
        puff_high = puff_bounds[1]  # upper bound on color

        frame = cv2.medianBlur(frame, 3)
        mask = cv2.inRange(frame, puff_low, puff_high)
        conts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_cont = sorted(conts, key=lambda x: x.size)[-1]

        # get the moments and find the center
        mmnts = cv2.moments(max_cont)
        cx = int(mmnts['m10'] / mmnts['m00'])
        cy = int(mmnts['m01'] / mmnts['m00'])

        return (cx, cy)

    def __get_puffs(self):
        """
        For each puff to be tracked, allow the user to specify the mask
        boundaries and see the results in real time.
        """
        num_puffs = input("How many puff balls to track? ")
        for puff in range(num_puffs):
            puff_low_limit, puff_high_limit = self.__get_puff_bounds()
            self.puffs.append([puff_low_limit, puff_high_limit])

    def __get_puff_bounds(self):
        """
        Open a window with the video feed and a mask applied which uses the
        input of the user through trackbars to control HSV values.
        :return: the lower and upper limits chosen by the user
        """
        done_choosing = False
        # original HSV filter values
        puff_low_limit = np.array([0, 140, 0], np.uint8)
        puff_hi_limit = np.array([179, 255, 255], np.uint8)

        def update_plh(x):
            puff_low_limit[0] = x

        def update_pls(x):
            puff_low_limit[1] = x

        def update_plv(x):
            puff_low_limit[2] = x

        def update_phh(x):
            puff_hi_limit[0] = x

        def update_phs(x):
            puff_hi_limit[1] = x

        def update_phv(x):
            puff_hi_limit[2] = x

        # Get first frame and mask it
        frame = self.get_raw_frame()
        # Show first frame
        cv2.imshow("Puff View", frame)
        # Add trackbar
        cv2.createTrackbar("Low Hue", "Puff View", 0, 179, update_plh)
        cv2.createTrackbar("Low Sat", "Puff View", 0, 255, update_pls)
        cv2.createTrackbar("Low Val", "Puff View", 0, 255, update_plv)
        cv2.createTrackbar("High Hue", "Puff View", 255, 179, update_phh)
        cv2.createTrackbar("High Sat", "Puff View", 255, 255, update_phs)
        cv2.createTrackbar("High Val", "Puff View", 255, 255, update_phv)

        while not done_choosing:
            frame = self.get_raw_frame()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frame = cv2.medianBlur(frame, 3)
            mask = cv2.inRange(frame, puff_low_limit, puff_hi_limit)
            frame = cv2.bitwise_and(frame, frame, mask=mask)
            frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
            cv2.imshow("Puff View", frame)
            k = cv2.waitKey(3) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break

        done_choosing = True if raw_input("Done (Y/N)").upper() == "Y" else False

        print(puff_low_limit, puff_hi_limit)
        return puff_low_limit, puff_hi_limit

    def shutdown(self):
        cv2.destroyAllWindows()
        self.capture.release()


if __name__ == '__main__':
    from time import sleep
    import sys

    if len(sys.argv) > 1:
        camNum = int(sys.argv[1])
        tracker = TensTracker(camNum, display=True, method=SUB, preset=False)
    else:
        tracker = TensTracker(display=True, method=PUFFS)

    while True:
        pass
    # tracker.shutdown()
