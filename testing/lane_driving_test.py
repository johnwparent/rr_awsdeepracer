from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
sys.path.append("..")
from clients import lane_finder
from clients import laneDriver

if __name__ == '__main__':

    url_servo = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://localhost:'+sys.argv[2]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)
    driver = laneDriver.LaneDrive(servo_ctrl)
    raw_input("Press Enter to begin: ")
    im = cam_data.getCurrentImage()
    im_ = laneDriver.WebcamImageToMat(im)
    driver.detect_lane(im_)
    driver.drive()
    servo_ctrl.Stop()
