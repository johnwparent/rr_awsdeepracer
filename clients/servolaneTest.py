from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
sys.path.append('..')
import laneDriver

def nd_arr_transform(ros_frame):
    _shape = (ros_frame.height,ros_frame.width,3)
    _dtype = np.uint8
    _buffer = ros_frame.data
    _offset = ros_frame.step
    _order = 'C'
    return np.ndarray(_shape,_dtype,_buffer,order=_order)

def image_stream(cam_data):
    im=cam_data.getCurrentImage()
    im_ = nd_arr_transform(im)
    return im_
    
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
    cam_ctrl.startCamera()
    driver = laneDriver.LaneDrive(None)
    while True:
        frame = image_stream(cam_ctrl)
        driver.detect_lane(frame)
        driver.drive()
        print(driver.c_drive_by_angle)