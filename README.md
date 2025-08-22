# network_camera_comm
Send camera stream by zmq msg from machineA to machineB, use lan or wlan instead of usb3.0 cable 

machine A should have enough power to deal with its slave cameras/sensors' data, and the network lantency should be as small as possible between machine A & B. 

Here we use rs 435i cameras, but the code is orient general.