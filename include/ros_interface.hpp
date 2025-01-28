#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/empty.h>
#include <sensor_msgs/msg/joint_state.h>
#include "st3215.hpp" // Include the header file for ST3215Controller

class MicroRosController {
public:
    MicroRosController();
    ~MicroRosController();

    // Setup function to initialize micro-ROS
    bool setup();

    // Run function to start the micro-ROS node
    void run();

    // Callback methods
    void feedbackTimerCallback();
    void jointStateCallback(const void* msgin);
    bool setHomePositionCallback(const void* req, void* res);

    int getAvailableServos();


    protected:
    // Helper methods

    bool syncTime();
    struct timespec getTime();

private:
    // micro-ROS entities
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    ST3215Controller mc;


    // Publishers
    rcl_publisher_t joint_state_publisher;

    // Subscribers
    rcl_subscription_t joint_state_subscriber;

    // Timers
    rcl_timer_t feedback_timer;

    // Service
    rcl_service_t service;

    // Message objects
    sensor_msgs__msg__JointState joint_state_msg;
    sensor_msgs__msg__JointState joint_msg_sub;
    std_srvs__srv__Empty_Request request_msg;
    std_srvs__srv__Empty_Response response_msg;

    // Robot state variables
    double current_position;
    double target_position;
    bool joint_msg_recieved;

    // Constants
    static const unsigned int FEEDBACK_TIMER_TIMEOUT = 50;
};


// Main.cpp needs to instantiate this variable
extern MicroRosController ros_controller;



#endif // ROS_INTERFACE_HPP