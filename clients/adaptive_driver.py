from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
from calibrationObj import calibrate_data
from camera_calibration import CameraCalibration
import laneDriver
import lane_finder
global lock,lane_driver,cam_data,servo_ctrl
lock = threading.RLock 


def april_avoid():
    7

#threaded main driver for our adaptive control
def drive_main():
    if not servo_ctrl.istf():
        lane_driver.drive()
    else:
        april_avoid()
        lane_driver.drive()



if __name__ == '__main__':
    lane_driver = laneDriver.LaneDrive()

    url_servo = 'rr+tcp://'+sys.argv[0]+':'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://'+sys.argv[0]+':'+sys.argv[1]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)

    
    #set up camera pipe stream
    p = cam_data.ImageStream.Connect(-1)
    p.PacketRecievedEvent+=next_frame


    #start camera
    try:
        cam_data.Start()
        servo_ctrl.setTf()
    except: pass

def next_frame(pipe_ep):
    global current_frame
    while(pipe_ep.Available > 0):
        image = pipe_ep.RecievePacket()
        current_frame = image
        drive_main()


