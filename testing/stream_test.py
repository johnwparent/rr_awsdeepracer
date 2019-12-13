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


    
current_frame = None

if __name__ == '__main__':
   
    url_camera = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=AWSCamera'
    cam_data = RRN.ConnectService(url_camera)
    p=cam_data.ImageStream.Connect(-1)
    p.PacketReceivedEvent+=next_frame
    raw_input("press enter to begin: ")
    cam_data.startCamera()
    while True:
        if (not current_frame is None):
            cv2.imshow("Image",current_frame)
        if cv2.waitKey(50)!=-1:
            break
    cv2.destroyAllWindows()
    p.Close()

def next_frame(pipe_ep):
    global current_frame
    while(pipe_ep.Available > 0):
        image = pipe_ep.RecievePacket()
        current_frame = nd_arr_transform(image)
    