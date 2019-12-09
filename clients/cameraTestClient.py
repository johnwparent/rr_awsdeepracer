from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
from cv_bridge import CvBridge, CvBridgeError
from camera_calibration import CameraCalibration


if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
    cam_ctrl.startCamera()
    cc = CameraCalibration()
    bridge = CvBridge()
    raw_input("Press_enter_to_capture_image: ")
    im = cam_ctrl.getCurrentImage()
    im = bridge.imgmsg_to_cv2(im, "bgr8")
    raw_input("Press_enter_to_capture_image: ")
    im2 = cam_ctrl.getCurrentImage()
    im2 = bridge.imgmsg_to_cv2(im2, "bgr8")
    cv2.imwrite("im1.png",im)
    cv2.imwrite("im2.png",im2)
    raw_input("Press enter to capture lane image: ")
    im3 = cam_ctrl.getCurrentImage()
    im3 = bridge.imgmsg_to_cv2(im3, "bgr8")
    cv2.imwrite("lane_image.png",im3)
    #cc.cal_main(cam_ctrl)
    
   
