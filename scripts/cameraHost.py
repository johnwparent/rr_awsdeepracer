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

class DeepRacerCamera_impl(object):
    def __init__(self, camera_name, mode, half_res):
        print "Initializing cam Node"
        rospy.init_node('aws_cams', anonymous = True)
        
        
        # Lock for multithreading
        self._lock = threading.RLock()
        self._camera_sub = False
        # for image pipe
        self._imagestream = None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()
        
        # set BaxterImage struct
        self._image = RR.RobotRaconteurNode.s.NewStructure("aws_interface.AWSImage")
        self._image.width = 0
        self._image.height = 0
        self._image.step = 0
        
    # open camera 
    def startCamera(self):
        
        print ("Subscribing to", self._camera_name)
        # get camera intrinsic values and fill Robot Raconteur struct
        self._caminfo_sub = rospy.Subscriber("/video_mjpeg", 
                                Image, self.setImageData)
        # Suscriber to camera image
        
        self._camera_sub = True
    
    
    # subscriber function for camera image
    def setImageData(self,camdata):
        with self._lock:
            if camdata.data:
                self._image.data = numpy.frombuffer(camdata.data,dtype="u1")
                self._image.height = camdata.height
                self._image.width = camdata.width
                self._image.step = camdata.step
                self._image.encoding = camdata.encoding
                self._image.is_bigendian = camdata.is_bigendian

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
    def getCurrentImage(self):
        with self._lock:
            return self._image
            
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
                return
def main():
    RR.RobotRaconteurNode.s.UseNumPy = True

    RR.RobotRaconteurNode.s.NodeName = "AWSCamService"

    print("registering services/pipes")
    obj = DeepRacerCamera_impl()
    with RR.ServerNodeSetup("AWSCamera_interface.AWSCamera",7788)
        RR.RobotRaconteurNode.s.RegisterServiceTypeFromFile("AWSCamera")
        RR.RobotRaconteurNode.s.RegisterService("AWSCamera","AWSCamera_interface.AWSCamera",obj)

        print("Service initiated")

        raw_input("Press enter to quit ...\r\n")

    #RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main()