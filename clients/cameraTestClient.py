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
    image1 = cc.reformat(cam_ctrl.getCurrentImage())
    image2 = cc.reformat(cam_ctrl.getCurrentImage())
    cv2.imshow("1st test",image1)
    cv2.imshow("2nd test",image2)

    cv2.waitKey()
    cv2.destroyAllWindows()
    
   