#!/usr/bin/env python
import roslib
roslib.load_manifest('rr_awsdeepracer')
import rospy
from std_msgs.sg import String
import time
import sys
import RobotRaconteur as RR
import thread 
import numpy
import traceback

from ctrl_msgs.msg import ServoCtrlMsg
from std_msgs.msg import Header
from std_msgs.msg import String

class RR_servo_impl(object):
    def __init__(self):
        print ("Initializing Node")
        self._pub = rospy.Publisher('/manual_drive',ServoCtrlMsg)
        rospy.init_node('rr_servo_host')   


    def Close(self):
        

    def setTurn(self, speed):
        
    def Stop(self):



    def Go(self):



    def Drive(self,vel,turn):


    def stopTurn(self):

def main():


    return

if __name__ == '__main__':
    main()