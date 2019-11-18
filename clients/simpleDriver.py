from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading




if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=Servo'
    print(url)
    print()
    print(RRN.GetRegisteredServiceTypes())
    print(RRN.IsServiceTypeRegistered('servo'))
    servo_ctrl = RRN.ConnectService(url)
    servo_ctrl.testing()
    servo_ctrl.Drive(0.8,0.1)
    time.sleep(2)
    servo_ctrl.Drive(0.4,0.1)
    time.sleep(4)
    servo_ctrl.Stop()
    
