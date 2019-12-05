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
    cv2.imshow(" ",im)
    cv2.imshow(" ",im2)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    
    cc.cal_main(cam_ctrl)


    cv2.waitKey()
    cv2.destroyAllWindows()
    
   
