from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
sys.path.append("..")
import lane_finder
import laneDriver

def nd_arr_transform(ros_frame):
    _shape = (ros_frame.height,ros_frame.width,3)
    _dtype = np.uint8
    _buffer = ros_frame.data
    _offset = ros_frame.step
    _order = 'C'
    return np.ndarray(_shape,_dtype,_buffer,order=_order)
    



if __name__ == '__main__':

    url_servo = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://localhost:'+sys.argv[2]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)
    driver = laneDriver.LaneDrive(servo_ctrl)
    cam_data.startCamera()
    raw_input("Press Enter to begin: ")
    im = cam_data.getCurrentImage()
    im_ = nd_arr_transform(im)
    driver.detect_lane(im_)
    driver.drive()
    time.sleep(6)
    servo_ctrl.Stop()
