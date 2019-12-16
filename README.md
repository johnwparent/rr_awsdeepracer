# Robot Raconteur to ROS AWS DeepRacer Bridge

This repo serves as the source code for the catkin package hosting the ROS Nodes running the [Robot Raconteur](https://robotraconteur.com) services. These services expose the servo control and the DeepRacer's image stream provided by it's camera. 
___
Collaborators: John Parent and Chukwuemeka Ike
___

## Getting started:
Installing this catkin package can be done by copying the root folder into a _catkin_ws/src_ folder within a ROS build. Then after starting ROS, running `$ catkin_make`  will build the catkin package and expose the RR services to ROS. Using GitHub, running `$ git clone https://github.com/johnwparent/rr_awsdeepracer.git` in the _catkin_ws/src_ directory and then running `catkin_make` will build the project in ROS. 

**Note: in order to build this catkin package, finding and sourcing the AWS _/servo_ctrl_ package on the DeepRacer is required in order to build the dependencies**

**Note: all scripts need to be made executable in order to be utilized by `rosrun`, so one needs to navigate into the scripts directory in a bash terminal and run**
```bash
chmod +x servoHost.py cameraHost.py
```

## Services
This RR - AWS ROS bridge wraps the DeepRacer's ROS controls with an RR abstraction, allowing for easier access to the DeepRacer's incredibly limited free control API from the client side. Two services are offered:

**servoHost.py** is responsible for controlling the servo responsible for driving the DeepRacer. The service is hosted and communicated via the localhost and TCP protocols, operating on port 2340.

**cameraHost.py** is responsible for providing the client side an image stream from the DeepRacer's front mounted camera. This service is accessed through the 2240 port.

___

Running both of these services is required to run the lane following client.

From two seperate bash terminals run:
```bash
$ rosrun rr_awsdeepracer [serviceFile]
```
inserting each service file name (listed above) into the [serviceFile ] command will initiate the services.

## Clients
Now that the services are running, the actual client to implement lane following can be initiated.
From the root folder of this catkin package, navigate into the clients folder. There, running the `ls` command will display a large suite of clients for interacting with the RR services. 

All but one of these clients are not needed to actually run the DeepRacer, but are used to demostrate the abilities of the RR ROS/AWS bridge, collect data, and to provide more basic control options, or to stream images.

The unnecessary clients are expanded upon further in the markdown file in that directory, but the required clients will be expanded upon now.

In order to run the lane follower, in the clients directory,
run:
```bash
$python lane_steam_test.py 2340 2240
```
The two values after the python file correspond to the tcp ports needed to access the RR services.

In the standard output, the user will be quried to press enter to begin the lane following. Place the DeepRacer in a location where it any solid white/dashed yellow lane lines are present, ensure that the camera and servo are on, and press enter. At this point the DeepRacer will navigate along the lane, keeping itself as close to the midpoint of the lane as possible, and will continue to drive until either *space* is pressed on the keyboard while focus is in the bash terminal running the client, or until the DeepRacer runs out of lanes and is no longer able to detect a road. At this point, the DeepRacer will output that 'no lane lines are detected' and will stop moving. 


## Testing 
The testing folder present in this package is designed to proccess and test video/image files, as well as to test the framerate of the DeepRacer RR stream, and generate the images utilized in the report. This folder is not necessary to run the project but if issues arise, is excellent for debugging. More detailed informating is provided in the revelant markdown file in the testing directory.