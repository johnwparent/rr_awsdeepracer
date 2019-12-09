import time
import numpy as np
import cv2
import sys
import threading
import logging


def iso_lines(frame):
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cv2.imwrite("hsv.png",hsv)
    ly = np.array([20,100,100],dtype=np.uint8)
    uy = np.array([30,255,255],dtype=np.uint8)
    lw = np.array([0,0,230],dtype=np.uint8)
    uw = np.array([255,255,255],dtype=np.uint8)
    w_mask = cv2.inRange(gray,200,255)
    #w_mask = cv2.inRange(hsv,lw,uw)
    y_mask = cv2.inRange(hsv,ly,uy)
    yw_mask = cv2.bitwise_or(w_mask,y_mask)
    cv2.imwrite("y mask.png",y_mask)
    cv2.imwrite("w mask.png", w_mask)
    cv2.imwrite("yw_mask.png",yw_mask)
    final_image = cv2.bitwise_and(gray,yw_mask)
    final_image_=cv2.GaussianBlur(final_image,(5,5),0)
    canny_edge = cv2.Canny(final_image_,200,600)
    return canny_edge,final_image

def ROI(frame):
    height, width = frame.shape
    mask = np.zeros_like(frame)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0.39 * width, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0.39*width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    roi = cv2.bitwise_and(frame, mask)
    cv2.imwrite("roi.png", roi)
    return roi

def find_lines(frame):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(frame, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)
    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]