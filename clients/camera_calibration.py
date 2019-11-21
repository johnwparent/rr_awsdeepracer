




from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
import threading
import calibrationObj

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
class CameraCalibration(object):

    def __init__(self):
        self._cameramatrix = []
        self._imagepoints = []
        self._taskspacepoints = []
        self._objp = np.zeros((6*8,3), np.float32)
	self._objp[:,:2]  =  np.mgrid[0:8,0:6].T.reshape(-1,2)

    def calibrate(self,img):
        self.ret,self.mtx,self.dist,self.rvecs,self.tvecs = cv2.calibrateCamera(self._taskspacepoints,self._imagepoints,img.shape[::-1],None,None)
        cali = calibrationObj.calibrate_data()
        cali.set_state(self.ret,self.mtx,self.dist,self.rvecs,self.tvecs)
        return cali
    def points(self, gray):
        ret, corners = cv2.findCheesboardCorners(gray, (7,6),None)
        if ret:
            self._taskspacepoints.append(self._objp)
            corners_ = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1),criteria)
            self._imagepoints.append(corners_)
            return self.calibrate(gray)

    def reformat(self, awsimg):
        frame = awsimg.data.reshape([awsimg.height,awsimg.width,3],order='C')
        return frame
    def undistort(self, obj,img):
        return cv2.undistort(img,obj.mtx,obj.dist,None,obj.mtx)

    def cal_main(self,cam):

        

        for x in range(15):
            #take image from RR
            print("deepracer capture #%i",x)
            input("Press enter for DeepRacer capture)
            #get rr image here
            #send for proccessing
            #todo: thread cause timing
            image_ = self.reformat(cam.GetCurrentImage())
            grayscale = cv2.cvtColor(image_,cv2.COLOR_BGR2GRAY)
            final = self.points(grayscale)
            print("image processed")
            return final

