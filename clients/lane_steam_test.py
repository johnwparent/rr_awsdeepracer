from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import logging
import keyboard
import Queue
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

def image_stream(cam_data):
    im=cam_data.getCurrentImage()
    im_ = nd_arr_transform(im)
    return im_
    
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
def main(frame):
    vid_im = nd_arr_transform(frame)
    out.write(vid_im)




class video_buffer(object):
    def __init__(self,driver):
        self._queue_lock = threading.RLock()
        self._queue_driver_lock = threading.RLock()
        self._queue = Queue.LifoQueue()
        self.driver = driver
        self.stopped = False    
    def add_queue(self,value):
        with self._queue_lock:
            self._queue.put(value)
        

    def thread_func(self):
        while not self.stopped and not self._queue.empty():
            with self._queue_driver_lock:
                frame = self._queue.get()
            self.driver.detect_lane(frame)
            self.driver.drive()




if __name__ == '__main__':

    url_servo = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=Servo'
    url_camera = 'rr+tcp://localhost:'+sys.argv[2]+'/?service=AWSCamera'
    servo_ctrl = RRN.ConnectService(url_servo)
    cam_data = RRN.ConnectService(url_camera)
    driver = laneDriver.LaneDrive(servo_ctrl)
    raw_input("press enter to begin: ")
    cam_data.startCamera()
    vb = video_buffer(driver)
    time.sleep(0.3)
    im = cam_data.getCurrentImage()
    frame_width = im.width
    frame_height = im.height
    #global out
    #out = cv2.VideoWriter('lane_real.avi',cv2.VideoWriter_fourcc(*'XVID'), 10, (frame_width,frame_height))
    servo_ctrl.Drive(0.8,0)
    while True:
        frame = image_stream(cam_data)
        vb.add_queue(frame)
        t = threading.Thread(target=vb.thread_func)
        t.start()
        #lane_lines = driver.lane_lines
        #lane_line_image = display_lines(frame,lane_lines)
        #out.write(lane_line_image)
        try:
            if keyboard.is_pressed('space'):
                servo_ctrl.Stop()
                break
        except:
            continue
    
    #out.release()
    