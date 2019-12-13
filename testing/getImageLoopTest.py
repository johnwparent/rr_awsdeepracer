from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
sys.path.append('..')
import threading
import logging


def nd_arr_transform(ros_frame):
    _shape = (ros_frame.height,ros_frame.width,3)
    _dtype = np.uint8
    _buffer = ros_frame.data
    _offset = ros_frame.step
    _order = 'C'
    return np.ndarray(_shape,_dtype,_buffer,order=_order)


if __name__ == '__main__':
   
    url_camera = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=AWSCamera'
    cam_data = RRN.ConnectService(url_camera)
    raw_input("press enter to begin: ")
    cam_data.startCamera()
    for i in range(6):
        counter = 0
        t_end = time.time() + 10
        while time.time() < t_end:
            im=cam_data.getCurrentImage()
            im_ = nd_arr_transform(im)
            counter+=1
        print("FPS: %i" % (counter//10))
