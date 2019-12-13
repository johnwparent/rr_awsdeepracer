import time
import numpy as np
import cv2
import sys
import threading
import logging
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


def main():
    path_to_video = sys.argv[1]
    cap = cv2.VideoCapture(path_to_video+'.avi')
    drive = laneDriver.LaneDrive(None)
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    global out 
    out = cv2.VideoWriter('lane_lines.avi',cv2.VideoWriter_fourcc(*'XVID'), 10, (frame_width,frame_height))
    while cap.isOpened():
        ret,frame = cap.read()
        drive.detect_lane(frame)
        lane_lines = drive.lane_lines
        lane_image = display_lines(frame,lane_lines,line_width=4)
        out.write(lane_image)

    cap.release()
    out.release()
if __name__ =='__main__':
    main()