from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading


if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
   