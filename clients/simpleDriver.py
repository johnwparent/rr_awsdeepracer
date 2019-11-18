from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading




if __name__ == '__main__':
    url = 'rr+tcp://'+sys.argv[1]+':'+sys.argv[2]+'?service=servo'
    print(url)
    print()
    print(RRN.GetRegisteredServiceTypes())
    print(RRN.IsServiceTypeRegistered('servo'))
    servo_ctrl = RRN.ConnectService(url)
    servo_ctrl.Go()
    time.sleep(1)
    servo_ctrl.Stop()
    
