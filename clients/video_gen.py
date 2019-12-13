from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
sys.path.append('..')

def main(frame):
    vid_im = nd_arr_transform(frame)
    out.write(vid_im)
    
def nd_arr_transform(ros_frame):
    _shape = (ros_frame.height,ros_frame.width,3)
    _dtype = np.uint8
    _buffer = ros_frame.data
    _offset = ros_frame.step
    _order = 'C'
    return np.ndarray(_shape,_dtype,_buffer,order=_order)


if __name__ == '__main__':
    url = 'rr+tcp://localhost:'+sys.argv[1]+'?service=AWSCamera'
    print(url)
    cam_ctrl = RRN.ConnectService(url)
    cam_ctrl.startCamera()
    
    im = cam_ctrl.getCurrentImage()
    frame_width = im.width
    frame_height = im.height
    global out 
    out = cv2.VideoWriter('lane.avi',cv2.VideoWriter_fourcc(*'XVID'), 10, (frame_width,frame_height))
    raw_input("Press_enter_to_start_video: ")
    start = time.time()
    while(True):
        im =cam_ctrl.getCurrentImage() 
        main(im)
    out.release()
