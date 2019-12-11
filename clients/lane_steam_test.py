from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
sys.path.append("..")
import lane_finder
import laneDriver

def nd_arr_transform(ros_frame):
    _shape = (ros_frame.height,ros_frame.width,3)
    _dtype = np.uint8
    _buffer = ros_frame.data
    _offset = ros_frame.step
    _order = 'C'
    return np.ndarray(_shape,_dtype,_buffer,order=_order)
    

def next_frame(pipe_ep):
    
    while(pipe_ep.Avaliable >0):
        image =pipe_ep.RecievePacket()
        current_frame = nd_arr_transform(image)
        driver.detect_lane(current_frame)
        driver.drive()

if __name__ == '__main__':

    url_servo = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://localhost:'+sys.argv[2]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)
    driver = laneDriver.LaneDrive(servo_ctrl)
    cam_data.startCamera()
    p=cam_data.ImageStream.Connect(-1)
    p.PacketRecievedEvent+=next_frame
    raw_input("Press Enter to begin: ")
    
    raw_input("Press Enter to end:")