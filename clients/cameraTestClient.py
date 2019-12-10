from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
from cv_bridge import CvBridge, CvBridgeError
from camera_calibration import CameraCalibration

def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
    cam_ctrl.startCamera()
    raw_input("Press_enter_to_capture_image: ")
    im = cam_ctrl.getCurrentImage()
    im_ = WebcamImageToMat(im)
    raw_input("Press_enter_to_capture_image: ")
    im2 = cam_ctrl.getCurrentImage()
    im2_ = WebcamImageToMat(im2)
    cv2.imwrite("im1.png",im_)
    cv2.imwrite("im2.png",im2_)
    raw_input("Press enter to capture lane image: ")
    im3 = cam_ctrl.getCurrentImage()
    im3_ = WebcamImageToMat(im3)
    cv2.imwrite("lane_image.png",im3_)
    #cc.cal_main(cam_ctrl)
    
   
