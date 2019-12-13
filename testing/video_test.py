import time
import numpy as np
import cv2
import sys
import threading
import logging
import math
sys.path.append('..')
from clients import lane_finder
from clients import laneDriver

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def main():
    path_to_video = sys.argv[1]
    cap = cv2.VideoCapture(path_to_video+'.avi')
    drive = laneDriver.LaneDrive(None)
    #frame_width = int(cap.get(3))
    #frame_height = int(cap.get(4))
    #global out 
    #out = cv2.VideoWriter('lane_lines_.avi',cv2.VideoWriter_fourcc(*'XVID'), 10, (frame_width,frame_height))
    while cap.isOpened():
        ret,frame = cap.read()
        drive.detect_lane(frame)
        #lane_image = display_lines(frame,lane_lines,line_width=4)
        drive.drive()
        print(drive.c_drive_by_angle*33.33)
        #lane_image_ = display_heading_line(lane_image,drive.c_drive_by_angle*33.33)
        #out.write(lane_image_)

    cap.release()
    #out.release()
if __name__ =='__main__':
    main()