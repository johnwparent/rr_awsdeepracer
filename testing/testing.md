# Testing the RR Bridge
In this directory are a number of clients and python scipts that can be used to test the Robot Raconteur services used to control the DeepRacer.

`getImageLoopTest.py`: must be run with the tcp port for the camera, and is used to test the FPS of the DeepRacer's RR image stream.

`video_test.py`: is used to test the ability of lane follower to correctly parse images and send servo commands

`stream_test.py`: tests the ability of the bridge to simply stream video

`lane_driving_test.py`: a one frame at a time, very slow lane follower used to test the ability of the lane follower to function, one frame every half second.

