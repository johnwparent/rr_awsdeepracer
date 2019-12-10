import time
import numpy as np
import cv2
import sys
sys.path.append('..')
import threading
from clients import lane_finder
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image




if __name__ == '__main__':
    frame = cv2.imread("lane_1.png")
    edges, ret = lane_finder.iso_lines(frame)
    cv2.imwrite('testing.png',ret)
    cv2.imwrite('edges.png',edges)
    roi = lane_finder.ROI(edges)
    lines = lane_finder.find_lines(roi)
    lines_image = display_lines(frame,lines)
    lines_mask_overlay = display_lines(ret,lines)
    cv2.imwrite("hTransform_mask.png",lines_mask_overlay)
    cv2.imwrite("hough_transform_lines.png",lines_image)
    lane_lines = lane_finder.average_slope_intercept(frame,lines)
    lane_lines_image = display_lines(frame,lane_lines)
    cv2.imwrite("lane_lines.png", lane_lines_image)
    