# UDP communication from ROS2 to ROS

This package is used to send poses from a machine running ROS2 to another machine running ROS1. 
A UDP server is used in the machine runnning ROS2.
The UDP server subscribes to a topic with a PoseArray message and sends the pose as an array to the UDP client. 
The UDP client is  used in the machine running ROS. 
The UDP client receive the array containing the poses by the UDP server and publishes a PoseStamped message in ROS. 

The first message sent is a "hello" message and it is only used to debug connection issues. 

## Building the package

To build the package in the machine running ROS2 (where the UDP server is supposed to run), after cloning the repository, follow these instructions.
- Rename the file *CMakeLists ament.txt* in *CMakeLists.txt*. 
- Rename the file *package ament.xml* in *package.xml*.
- Compile the package using: 
```
colcon build --packages-select ros2_udp
```

To build the package in the machine with ROS (where the UDP client is supposed to run), after cloning the repository, follow these instructions. 
- Rename the file *CMakeLists catkin.txt* in *CMakeLists.txt*. 
- Rename the file *package catkin.xml* in *package.xml*. 
- Compile the package using: 
```
catkin_make 
```

## Run 

To run the UDP server, in the machine running ROS2, type: 
```
ros2 run ros2_udp udp_server
```
To run the UDP client, in the machine running ROS, type: 
```
rosrun ros2_udp udp_client
```