#!/usr/bin/env python
import roslib
roslib.load_manifest('rr_awsdeepracer')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import time
import sys
import RobotRaconteur as RR
import thread 
import threading
import numpy
import traceback
import cv2
service_def = """
service AWSCamera_interface
stdver 0.9

struct AWSImage
    field int32 width
    field int32 height
    field int32 step
    field uint8 is_bigendian
    field string encoding
    field uint8[] data
end

object AWSCamera
    # camera control functions
    function void startCamera()
    
    # functions to acquire data on the image
    function AWSImage getCurrentImage()
    
    # pipe to stream images through
    pipe AWSImage ImageStream
    
end
"""


class DeepRacerCamera_impl(object):
    """Amazon AWS Deepracer Service Object for `Robot Raconteur`_.
    .. _`Robot Raconteur`: http://robotraconteur.com

    

    """
    def __init__(self):
        print ("Initializing cam Node")
        rospy.init_node('aws_cams', anonymous = True)
        
        self._imagestream_endpoints = {}
        self._imagestream_endpoints_lock=threading.RLock()
        # Lock for multithreading
        self._lock = threading.RLock()
        self._camera_sub = False
        # for image pipe
        self._imagestream = None
        
       
    def set_image_struct(self):
        # set aws struct
        self._image = RR.RobotRaconteurNode.s.NewStructure("AWSCamera_interface.AWSImage")
        self._image.width = 0
        self._image.height = 0
        self._image.step = 0
    # open camera 
    def startCamera(self):
        
        print ("Subscribing to AWSCamera: Rectified image")
        # get camera intrinsic values and fill Robot Raconteur struct
        self._caminfo_sub = rospy.Subscriber("/cv_camera/image_rect_color", 
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
                frame = self._image
                self._imagestream_broadcaster.AsyncSendPacket(frame,lambda:None)
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
        print("image_stream set")
        print(value)
        # Set the PipeConnecCallback to ImageStream_pipeconnect that will
        # called when a PipeEndpoint connects
        self._imagestream_broadcaster=RR.PipeBroadcaster(value,3)
    

    
def main():
    RR.RobotRaconteurNode.s.UseNumPy = True

    #RR.RobotRaconteurNode.s.NodeName = "AWSCamService"
    with RR.ServerNodeSetup("AWSCamera_interface.AWSCamera",2240):
        print("registering services/pipes")
        obj = DeepRacerCamera_impl()


        RR.RobotRaconteurNode.s.RegisterServiceType(service_def)
        RR.RobotRaconteurNode.s.RegisterService("AWSCamera","AWSCamera_interface.AWSCamera",obj)
        obj.set_image_struct()
        print("Service initiated")

        raw_input("Press enter to quit ...\r\n")

        #RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main()