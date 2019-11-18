from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
from calibrationObj import calibrate_data
from camera_calibration import CameraCalibration

global cam_calibration, calibrate



class LaneDrive(object):
    def __init__(self,cam_obj):
        self._calibrationObj = cam_obj
    def drive_by_angle(self):
        #todo implement this
        self._none = None

def next_frame(pipe_ep):
    global current_frame

    while(pipe_ep.Avaliable >0):
        image =pipe_ep.RecievePacket()
        tmp_img = calibrate.reformat(image)
        current_image = calibrate.undistort(cam_calibration,tmp_img)
        current_frame = current_image

if __name__ == '__main__':
    calibrate = CameraCalibration()

    #connect to RR services needed to drive and get data
    url_servo = 'rr+tcp://'+sys.argv[0]+':'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://'+sys.argv[0]+':'+sys.argv[1]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)

    
    #set up camera pipe stream
    p = cam_data.ImageStream.Connect(-1)
    p.PacketRecievedEvent+=next_frame

    while True:
        #Just loop resetting the frame
        #This is not ideal but good enough for demonstration

        if (not current_frame is None):
            calibrate.undistort(cam_calibration,current_frame)
        if cv2.waitKey(50)!=-1:
            break

    #start camera
    try:
        cam_data.Start()
    except: pass
    #calibrate camera
    
    cam_calibration = calibrate.cal_main(sys.argv)
