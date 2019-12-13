from RobotRaconteur.Client import *
import time
import numpy as np
import cv2
import sys
sys.path.append('..')
import threading
import logging

def next_frame(pipe_ep):
    
    while(pipe_ep.Avaliable > 0):
        print("recieved frame")
        image =pipe_ep.RecievePacket()
        print(image.height)


if __name__ == '__main__':
    url_camera = 'rr+tcp://localhost:'+sys.argv[1]+'/?service=AWSCamera'
    cam_data = RRN.ConnectService(url_camera)
    p=cam_data.ImageStream.Connect(-1)
    print(p)
    p.PacketReceivedEvent+=next_frame
    raw_input("press enter to begin: ")
    cam_data.startCamera()


    raw_input("Press enter to end: ")
    p.Close()