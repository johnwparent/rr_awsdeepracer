#!/usr/bin/env python
import roslib
roslib.load_manifest('rr_awsdeepracer')
import rospy
from std_msgs.sg import String
from std_msgs.sg import Header
from sensor_msgs.msg import Image
import time
import sys
import RobotRaconteur as RR
import thread 
import numpy
import traceback
import cv2;
import cv2.aruco as aruco

class BaxterCamera_impl(object):
    def __init__(self, camera_name, mode, half_res):
        print "Initializing ROS Node"
        rospy.init_node('baxter_cameras', anonymous = True)
        
        
        # Lock for multithreading
        self._lock = threading.RLock()
        
        # for image pipe
        self._imagestream = None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()
        
        # get access to camera controls from RSDK
        self._camera = baxter_interface.CameraController(camera_name)
        self._camera_name = camera_name;
        
        # automatically close camera at start
        # self._camera.close()
        self._camera_open = False
        
        # set constant ImageHeader structure
        self.setResolution(mode,half_res)
        self._image_header = RR.RobotRaconteurNode.s.NewStructure( 
                                    "BaxterCamera_interface.ImageHeader" )
        self._image_header.width = int(self._camera.resolution[0])
        self._image_header.height = int(self._camera.resolution[1])
        self._image_header.step = int(4)
        
        
        self._camera_intrinsics = None
        
        # set exposure, gain, white_balance to auto
        self._camera.exposure = self._camera.CONTROL_AUTO
        self._camera.gain = self._camera.CONTROL_AUTO
        self._camera.white_balance_red = self._camera.CONTROL_AUTO
        self._camera.white_balance_green = self._camera.CONTROL_AUTO
        self._camera.white_balance_blue = self._camera.CONTROL_AUTO
        
        # set BaxterImage struct
        self._image = RR.RobotRaconteurNode.s.NewStructure("BaxterCamera_interface.BaxterImage")
        self._image.width = self._image_header.width
        self._image.height = self._image_header.height
        self._image.step = self._image_header.step

        # Initialize ARtag detection
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self._arucoParams = aruco.DetectorParameters_create()
        self._markerSize = 0.085
        
    # open camera 
    def openCamera(self):
        if self._camera_open:
            return
        
        # start camera and subscription
        try:
            self._camera.open()
        except (OSError):
            print "Could not open camera.  Perhaps the other cameras are already open?"
            return
        
        # get camera intrinsic values and fill Robot Raconteur struct
        self._caminfo_sub = rospy.Subscriber("cameras/" + self._camera_name + "/camera_info", 
                                CameraInfo, self.set_CameraIntrinsics)
        # Suscriber to camera image
        print "Subscribing to", self._camera_name
        self._image_sub = rospy.Subscriber("cameras/" + self._camera_name + "/image", Image, self.set_imagedata) 
        self._camera_open = True
    
    
    def closeCamera(self):
        if not self._camera_open:
            return
    
        if self._image_sub:
            self._image_sub.unregister()
        self._camera.close()
        self._camera_open = False
    
    @property
    def camera_open(self):
        if self._camera_open:
            return 1
        else:
            return 0
    
    # subscriber function for camera image
    def set_imagedata(self,camdata):
        with self._lock:
            if camdata.data:
                self._image.data = numpy.frombuffer(camdata.data,dtype="u1")
        
        with self._imagestream_endpoints_lock:
            # Send to pipe endpoints
            for ep in self._imagestream_endpoints:
                dict_ep = self._imagestream_endpoints[ep]
                # Loop through indices in nested dict
                for ind in dict_ep:
                    # Attempt to send frame to connected endpoint
                    try:
                        pipe_ep=dict_ep[ind]
                        pipe_ep.SendPacket(self._image)
                    except:
                        # on error, assume pipe has been closed
                        self.ImageStream_pipeclosed(pipe_ep)
    
    def set_CameraIntrinsics(self, data):
        if (self._camera_intrinsics is None):
            print "Setting Camera Intrinsic Data"
            self._camera_intrinsics = RR.RobotRaconteurNode.s.NewStructure( 
                                        "BaxterCamera_interface.CameraIntrinsics" )
            K = list(data.K)
            K[2] -= data.roi.x_offset;
            K[5] -= data.roi.y_offset;
            self._camera_intrinsics.K = tuple(K)
            self._camera_intrinsics.D = tuple(data.D)
            self._caminfo_sub.unregister()

    # The following function is to set camera parameters manually
    def setCameraIntrinsics(self, data):
        if (self._camera_intrinsics is None):
            print "Setting Camera Intrinsic Data"
        else:
            print "Setting already exists. Overwriting now..."
        K = list(data.K)
        self._camera_intrinsics.K = tuple(K)
        self._camera_intrinsics.D = tuple(data.D)
        self._caminfo_sub.unregister()
            
    def getCurrentImage(self):
        with self._lock:    
            return self._image
    
    def getImageHeader(self):
        return self._image_header
    
    def getCameraIntrinsics(self):
        return self._camera_intrinsics
        
    ''' This is meant to only be called once at the initialization of the program'''
    def setResolution(self, mode, half_res):
        
        self._camera.resolution = self._camera.MODES[mode]
        
        # half resolution not always possible
        if (mode in [0,1,4] and half_res == 1):
            print 'Cannot do half-resolution at (1280,800), (960, 600), or (384,240)'
            half_res = 0
        self._camera.half_resolution = (half_res != 0)
        
        print 'Resolution set to: ', self._camera.resolution
        if (self._camera.half_resolution):
            print '**Displaying at half-resolution'
    
    def setExposure(self, exposure):
        if (exposure < 0 or exposure > 100 and exposure != self._camera.CONTROL_AUTO):
            print 'Exposure must be in [0, 100]'
            return
        self._camera.exposure = exposure
    
    def setGain(self, gain):
        if (gain < 0 or gain > 79 and gain != self._camera.CONTROL_AUTO):
            print 'Gain must be in [0, 79]'
            return
        self._camera.gain = gain
    
    def setWhiteBalance(self, red, green, blue):
        if (red < 0 or red > 4095 and red != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_red = red
        
        if (green < 0 or green > 4095 and green != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_green = green
        
        if (blue < 0 or blue > 4095 and blue != self._camera.CONTROL_AUTO):
            print 'White Balance values must be in [0, 4095]'
            return
        self._camera.white_balance_blue = blue
    
    def setFPS(self, fps):
        if (fps <= 0 or fps > 30):
            print 'fps must be positive and cannot exceed 30'
            return
        self._camera.fps = fps

    # Functions related to AR tags
    # Marker size
    def setMarkerSize(self, markerSize):
        with self._lock:
            self._markerSize = markerSize

    def getMarkerSize(self):
        with self._lock:
            markerSize = self._markerSize
            return markerSize

    def ARtag_Detection(self):
        if not self.camera_open:
            self.openCamera()
        print "Detecting AR tags..."
        currentImage = self.getCurrentImage()
        imageData = currentImage.data
        imageData = numpy.reshape(imageData, (800, 1280, 4))
        gray = cv2.cvtColor(imageData, cv2.COLOR_BGRA2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._arucoParams) 

        if ids is not None:
            Tmat = []
            IDS = []
            detectioninfo = RR.RobotRaconteurNode.s.NewStructure("BaxterCamera_interface.ARtagInfo")
            for anid in ids:
                IDS.append(anid[0])
            for corner in corners:
                pc, Rc = self.getObjectPose(corner) 
                Tmat.extend([   Rc[0][0],   Rc[1][0],   Rc[2][0],   0.0,
                                Rc[0][1],   Rc[1][1],   Rc[2][1],   0.0,
                                Rc[0][2],   Rc[1][2],   Rc[2][2],   0.0,
                                pc[0],      pc[1],      pc[2],      1.0])

            detectioninfo.tmats = Tmat
            detectioninfo.ids = IDS
            return detectioninfo

    # function that AR tag detection uses
    def getObjectPose(self, corners):
        with self._lock:
            camMatrix = numpy.reshape(self._camera_intrinsics.K, (3, 3))
            
            distCoeff = numpy.zeros((1, 5), dtype=numpy.float64)
            distCoeff[0][0] = self._camera_intrinsics.D[0]
            distCoeff[0][1] = self._camera_intrinsics.D[1]
            distCoeff[0][2] = self._camera_intrinsics.D[2]
            distCoeff[0][3] = self._camera_intrinsics.D[3]
            distCoeff[0][4] = self._camera_intrinsics.D[4]

            # print "cameramatrix: ", camMatrix
            # print "distortion coefficient: ", distCoeff

            # AR Tag Dimensions in object frame
            objPoints = numpy.zeros((4, 3), dtype=numpy.float64)
            # (-1, +1, 0)
            objPoints[0,0] = -1*self._markerSize/2.0 # -1
            objPoints[0,1] = 1*self._markerSize/2.0 # +1
            objPoints[0,2] = 0.0
            # (+1, +1, 0)
            objPoints[1,0] = self._markerSize/2.0 # +1
            objPoints[1,1] = self._markerSize/2.0 # +1
            objPoints[1,2] = 0.0
            # (+1, -1, 0)
            objPoints[2,0] = self._markerSize/2.0 # +1
            objPoints[2,1] = -1*self._markerSize/2.0 # -1
            objPoints[2,2] = 0.0
            # (-1, -1, 0)
            objPoints[3,0] = -1*self._markerSize/2.0 # -1
            objPoints[3,1] = -1*self._markerSize/2.0 # -1
            objPoints[3,2] = 0.0

            # Get each corner of the tags
            imgPoints = numpy.zeros((4, 2), dtype=numpy.float64)
            for i in range(4):
                imgPoints[i, :] = corners[0, i, :]

            # SolvePnP
            retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camMatrix, distCoeff)
            Rca, b = cv2.Rodrigues(rvec)
            Pca = tvec

            # print "pca, rca: ", Pca, Rca

            return [Pca, Rca]


    ######################################

    # pipe functions
    @property
    def ImageStream(self):
        return self._imagestream
    
    @ImageStream.setter
    def ImageStream(self, value):
        self._imagestream = value
        # Set the PipeConnecCallback to ImageStream_pipeconnect that will
        # called when a PipeEndpoint connects
        value.PipeConnectCallback = self.ImageStream_pipeconnect
    
    def ImageStream_pipeconnect(self, pipe_ep):
        # Lock the _imagestream_endpoints deictionary and place he pipe_ep in 
        # a nested dict that is indexed by the endpoint of the client and the 
        # index of the pipe
        with self._imagestream_endpoints_lock:
            # if there is not an enry for this client endpoint, add it
            if (not pipe_ep.Endpoint in self._imagestream_endpoints):
                self._imagestream_endpoints[pipe_ep.Endpoint] = dict()
            
            # Add pipe_ep to the correct dictionary given the endpoint + index
            dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
            dict_ep[pipe_ep.Index] = pipe_ep
            pipe_ep.PipeEndpointClosedCallback = self.ImageStream_pipeclosed
    
    def ImageStream_pipeclosed(self, pipe_ep):
        with self._imagestream_endpoints_lock:
            try:
                dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
                del(dict_ep[pipe_ep.Index])
            except:
                traceback.print_exc()

if __name__ == '__main__':
    