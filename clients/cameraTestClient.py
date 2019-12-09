from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
from camera_calibration import CameraCalibration


if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
    cam_ctrl.startCamera()
    cc = CameraCalibration()
    raw_input("Press_enter_to_capture_image: ")
    im = cam_ctrl.getCurrentImage()
    raw_input("Press_enter_to_capture_image: ")
    im2 = cam_ctrl.getCurrentImage()
    cv2.imwrite("im1.png",im.data)
    cv2.imwrite("im2.png",im2.data)
    raw_input("Press enter to capture lane image: ")
    im3 = cam_ctrl.getCurrentImage()
    cv2.imwrite("lane_image.png",im3.data)
    #cc.cal_main(cam_ctrl)
    
   
