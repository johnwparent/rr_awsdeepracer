import time
import numpy as np
import cv2
import sys
import math
import threading
import logging
#######################REMOVE THE CLIENT IMPORT ##################
from clients import lane_finder


global cam_calibration, calibrate

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle

class LaneDrive(object):
    def __init__(self,servo_ctrl):
        self._speed = 0
        self._turn = 0
        self._obj = False
        self._servo = servo_ctrl
        self.c_drive_by_angle = 0.0
    def drive(self):
        if len(self._lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            return

        new_steering_angle = compute_steering_angle(self._frame, self._lane_lines)
        self.c_drive_by_angle = stabilize_steering_angle(self.c_drive_by_angle, new_steering_angle, len(self._lane_lines))/33.33
        if self._servo is not None:
            self._servo.Drive(0.65,self.c_drive_by_angle)
        


    def detect_lane(self,frame):
        edges,frame_final = lane_finder.iso_lines(frame)
        roi = lane_finder.ROI(edges)
        lines = lane_finder.find_lines(roi)
        lane_lines = lane_finder.average_slope_intercept(frame,lines)
        self._lane_lines = lane_lines
        self._frame = frame
    @property
    def lane_lines(self):
        return self._lane_lines
    @property
    def frame(self):
        return self._frame
global current_frame
global driver
def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2
def next_frame(pipe_ep):
    
    while(pipe_ep.Avaliable >0):
        image =pipe_ep.RecievePacket()
        current_frame = WebcamImageToMat(image)
        driver.detect_lane(current_frame)
        driver.drive()


