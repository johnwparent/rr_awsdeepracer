#!/usr/bin/env python
import roslib
roslib.load_manifest('rr_awsdeepracer')
import rospy
from std_msgs.msg import String
import time
import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import thread 
import numpy
import traceback

from ctrl_pkg.msg import ServoCtrlMsg
from std_msgs.msg import Header
from std_msgs.msg import String
service_def ="""
service servo
stdver 0.9
object Servo
    property double servo_velocity
    property double serv_torque
    function void Stop()
    function void Close()
    function void setTurn(double speed)
    function void Go()
    function void Drive(double speed, double turn)
    function void stopTurn()
end
"""


class RR_servo_impl(object):
    def __init__(self):
        print ("Initializing Node")
        self._pub = rospy.Publisher('/manual_drive',ServoCtrlMsg)
        rospy.init_node('rr_servo_host')   


        self._angle = 0.0
        self._throttle = 0.0
        self._senderStop = ServoCtrlMsg()
        self._senderStop.angle = 0.0
        self._senderStop.throttle = 0.0
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
        RRN.RegisterServiceType(service_def)
        RRN.RegisterService("Servo","servo.Servo",obj)

        raw_input("Server_started,_press_enter_to_quit...")

        obj.Close()

    return

if __name__ == '__main__':
    main()