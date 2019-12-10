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
    function void testing()
    function bool istf()
    function tsData gettf()
end
"""


class RR_servo_impl(object):
    def __init__(self):
        print ("Initializing Node")
        self._pub = rospy.Publisher('/manual_drive',ServoCtrlMsg,queue_size=50)
        rospy.init_node('rr_servo_host')   
        # self._tfInfo = RR.RobotRaconteurNode.s.NewStructure("servo.tsData")
        # self._tfInfo.dist = 0
        # self._tfInfo.orientation_x = 0
        # self._tfInfo.orientation_z = 0
        # self._isaprilTag = False
        self._angle = 0.0
        self._throttle = 0.0
        self._senderStop = ServoCtrlMsg()
        self._senderStop.angle = 0.0
        self._senderStop.throttle = 0.0
        self._senderGo = ServoCtrlMsg()
        self._senderGo.angle = 0.0
        self._senderGo.throttle = 0.3

        self._currenttf = None
        self._oldtf = None

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
    def testing(self):
        rospy.loginfo("The rr servo service can communicate!!!")

    # def gettf(self):
    #     self._tfInfo.dist = self._currenttf.pose.position.z
    #     self._tfInfo.orientation_x = self.currenttf.pose.position.x 
    #     self._tfInfo.orientation_z = self._currenttf.pose.postion.z
        
    #     return self._tfInfo


    # def istf(self):
    #     return self._isaprilTag and self.tf_diff()

    # def tf_diff(self):
    #     return not (self._currenttf == self._oldtf)
    # def tf(self,aprildata=None):
    #     self._isaprilTag = True
    #     self._oldtf = self._currenttf
    #     self._currenttf = aprildata
    # def setTf(self):
    #     self._subtf = rospy.Subscriber("/tf",AprilTagDetection, self.tf)
        

def main():
    obj = RR_servo_impl()
    with RR.ServerNodeSetup("servo.Servo",2340):
        RRN.RegisterServiceType(service_def)
        RRN.RegisterService("Servo","servo.Servo",obj)
        print(2340)
        raw_input("Server_started,_press_enter_to_quit...")

        obj.Close()

    return

if __name__ == '__main__':
    main()