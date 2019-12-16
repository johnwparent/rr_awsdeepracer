# Robot Raconteur Services
There are two RR services located in this directory, `servoHost.py` and `cameraHost.py` responsible for running the Robot Raconteur interface for the DeepRacer control API.

Additionally the `.robdef` files required to run the services, and expose the services control API are present. These `.robdef`s list the method available to a client connected to either service. 

Running the services is detailed in the root level README, but further information is necessary to generate future clients.

The servo service runs off: 
```python
'rr+tcp://localhost:'+sys.argv[1]+'/?service=Servo'
```
and the camera service:
```python
'rr+tcp://localhost:'+sys.argv[2]+'/?service=AWSCamera'
```
These are the commands needed to connect clients to the DeepRacer's service. The system arguments correspond to the tcp port ids entered as command line arguments to the clients, actual ordering is up to a user of course.