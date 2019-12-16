# RR Bridge Clients

The necessary client to run the service is detailed in the README. 
`lane_finder.py` and `laneDriver.py` are used by the main client to provide image processing, and lane navigation and control, all `lane_steam_test.py` actually does is generate a thread pool to send images to the image processor and driver.

Other important clients are the `simpleDriver.py` which can be used to demonstrate the systems ability to control the DeepRacer in an incredibly basic context, and the systems ability to function in the first place.

`cameraTestClient.py` provides a basic image stream to demonstrate and test the bridge's ability to connect to the DeepRacer's camera, and provide at least a 10 FPS stream to the client.

All others can be experiemnted with at will, and all should be fully functional.