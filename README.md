# st3215_microros

Repo for controlling a SO-100 arm using the Waveshare ESP32 served driver board: https://www.waveshare.com/servo-driver-with-esp32.htm 



# Getting started 

Use platform.io to compile and upload to your ESP-32 chip. 

## Config platformio.ini for you system.

Open `platformio.ini` and look at the build flags. We need to modify these to suit your robot. 

Change the JOINT_IDS to reflect what motor ids are in the joints of your arm. Note that you need to get the order the same as the joint names list above. 

Make sure to set your WIFI SSID and password in `platformio.ini`, as well as the ip address of your agent computer. 


## Run Micro Ros Agent

Next run the ros agent on your computer (you will need the `micro_ros_agent` package installed): 

`ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 6`


## Get feedback

You should see feedback from your robot arm on the `joint_states` topic. 


## Control the arm 

You should be able to send desired joint states to your motors by publishing a JointState message to `/desired_joint_states` topic. 

## ROS2 Control 

To get ROS2 control working, use the topic_based ros2 control node here: 

https://github.com/PickNikRobotics/topic_based_ros2_control



