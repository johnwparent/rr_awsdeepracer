import time
import numpy as np
import cv2


class calibrate_data(object):
    def __init__(self):
        self.ret = None
        self.mtx = []
        self.dist = 0
        self.rvecs = []
        self.tvecs = []
    def set_state(self,ret,mtx,dist,rvecs,tvecs):
        self.ret = ret
        self.mtx = mtx
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs