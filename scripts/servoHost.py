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

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

class RR_servo_impl(object):
    def __init__(self):
        print ("Initializing Node")
        rospy.init_node('rr_servo_host')
                      
        # Start background threads
        self._running = True
        self._t_joints = threading.Thread(target=self.jointspace_worker)
        self._t_joints.daemon = True
        self._t_joints.start()
        
        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()

    def close(self):
        self._running = False
        self._t_joints.join()
        self._t_effector.join()
        self._t_command.join()
        
        if (self._mode != self.MODE_POSITION):
            self._left.exit_control_mode()
            self._right.exit_control_mode()

    def setPositionModeSpeed(self, speed):
        if speed < 0.0:
            speed = 0.0
        elif speed > 1.0:
            speed = 1.0
        
        self._left.set_joint_position_speed(speed)
        self._right.set_joint_position_speed(speed)
        
def main():
    return

if __name__ == '__main__':
    main()