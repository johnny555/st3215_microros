#include "display.hpp"
#include "ros_interface.hpp"
#include "st3215.hpp"

// Public variable!!
MicroRosController ros_controller;

void setup() {
    setup_display();
    
    if (!ros_controller.setup()) {
        print_msg("ROS setup failed!");
        while(1) { delay(1000); }
    }
    int n_servos = ros_controller.getAvailableServos();
    status.num_motors = n_servos;
    print_msg("Servos Configured");

    status.ip_address = WiFi.localIP().toString();
    print_msg("ROS Configured");
    
}

void loop() {
    ros_controller.run();
}

