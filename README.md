# st3215_microros

Repo for controlling a SO-100 arm using the Waveshare ESP32 served driver board: https://www.waveshare.com/servo-driver-with-esp32.htm 



## Getting started 

Use platform.io to compile and upload to your ESP-32 chip. 

Make sure to set your WIFI SSID and password in `platform.io`, as well as the ip address of your agent computer. 

Then run the ros agent on your computer (you will need the `micro_ros_agent` package installed): 

ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 6