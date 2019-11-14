#!/usr/bin/env python
import roslib
roslib.load_manifest('rr_awsdeepracer')
import rospy
from std_msgs.msg import String
import time
import sys
import RobotRaconteur as RR
import thread 
import numpy
import traceback

from ctrl_pkg.msg import ServoCtrlMsg
from std_msgs.msg import Header
from std_msgs.msg import String

class RR_servo_impl(object):
    def __init__(self):
        print ("Initializing Node")
        self._pub = rospy.Publisher('/manual_drive',ServoCtrlMsg)
        rospy.init_node('rr_servo_host')   


        self._angle = 0.0
        self._throttle = 0.0
        self._senderStop = ServoCtrlMsg()
        self._sender.angle = 0.0
        self._sender.throttle = 0.0
        self._senderGo = ServoCtrlMsg()
        self._senderGo.angle = 0.0
        self._senderGo.throttle = 0.1


    @property
    def servo_velocity(self):
        return self._throttle

    @property
    def serv_torque(self):
        return self._turn
    #close the rr service
    def Close(self):
        RR.RobotRaconteurNode.s.Shutdown()

    def setTurn(self, speed):
        msg = ServoCtrlMsg()
        msg.angle = speed
        msg.throttle = self._throttle
        self._pub.publish(msg)

    def Stop(self):
        self._pub.publish(self._senderStop)

    def Go(self):
        self._pub.publish(self._senderGo)


    def Drive(self,vel,turn):
        msg = ServoCtrlMsg()
        msg.angle = turn
        msg.throttle = vel
        self._throttle = vel
        self._turn = turn
        self._pub.publish(msg)

    def stopTurn(self):
        msg = ServoCtrlMsg()
        msg.angle = 0.0
        msg.throttle = self._throttle
        self._pub.publish(msg)


def main():
    obj = RR_servo_impl()
    with RR.ServerNodeSetup("servo",2340):
        RRN.RegisterServiceTypeFromFile("servo")
        RRN.RegisterService("Servo","servo.Servo",obj)

        raw_input("Server_started,_press_enter_to_quit...")

        obj.Close()

    return

if __name__ == '__main__':
    main()