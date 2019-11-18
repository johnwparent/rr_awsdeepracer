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
    servo_ctrl.Go()
    servo_ctrl.Drive(0.5,0)
    time.sleep(1)
    servo_ctrl.Stop()
    
