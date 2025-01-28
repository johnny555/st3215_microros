#include "ros_interface.hpp"
#include "display.hpp"
#include "jointstate.h"
#include "st3215.hpp"
#include <micro_ros_platformio.h>

MicroRosController::MicroRosController() : 
    joint_msg_recieved(false),
    current_position(0),
    target_position(0) {
    allocator = rcl_get_default_allocator();
}

MicroRosController::~MicroRosController() {
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_subscription_fini(&joint_state_subscriber, &node);
    rcl_service_fini(&service, &node);
    rcl_timer_fini(&feedback_timer);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

void MicroRosController::feedbackTimerCallback() {

        if (joint_msg_recieved) {
            int id;
            for (int i = 0; i < joint_msg_sub.name.size; ++i) {
                id = mc.getMotorId(joint_msg_sub.name.data[i].data);            
                mc.moveMotor(id, joint_msg_sub.position.data[i]);
            }
        }
        std::vector<ServoData> servoData = mc.getFeedbackAll();

        auto ts = getTime();

        joint_state_msg.header.stamp.sec = ts.tv_sec;
        joint_state_msg.header.stamp.nanosec = ts.tv_nsec;
        
        for (int i = 0; i < servoData.size(); i++) {
            joint_state_msg.position.data[i] = servoData[i].position;
            joint_state_msg.velocity.data[i] = servoData[i].speed;
            joint_state_msg.effort.data[i] = servoData[i].load;
            strcpy(joint_state_msg.name.data[i].data, servoData[i].name.c_str());
        }
        rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);

}

void MicroRosController::jointStateCallback(const void* msgin) {
    joint_msg_recieved = true;
}

bool MicroRosController::setHomePositionCallback(const void* req, void* res) {
    mc.setMiddleAll();
    print_msg("Home position set");
    return true;
}

int MicroRosController::getAvailableServos(){
    return mc.getAvailableServos();
}

// Dodgy hack to allow callbacks that can call methods of a class. Requires that the variable
// ros_controller is a pointer to an instance of the class MicroRosController in the global space.
// Assumed to be constructed by main.cpp



void feedback_timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time) {

        ros_controller.feedbackTimerCallback();
}

void joint_state_callback_wrapper(const void* msg) {
        ros_controller.jointStateCallback(msg);
}

void set_home_position_callback_wrapper(const void* req, void* res) {
        ros_controller.setHomePositionCallback(req, res);
}



bool MicroRosController::setup() {
    mc.initialize();
    IPAddress agent_ip(AGENT_IP);
    size_t agent_port = 8888;

    char msg[100];
    snprintf(msg, sizeof(msg), "ROS Wifi:\nConnecting to agent:\n %s\n %s", agent_ip.toString().c_str(), SECRET_SSID);
    print_msg(msg);

    set_microros_wifi_transports(SECRET_SSID, SECRET_PASSWORD, agent_ip, agent_port);
    
    print_msg("setup: connected.");

    // Initialize support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Setup joint state messages
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 25;
    conf.max_ros2_type_sequence_capacity = 10;
    conf.max_basic_type_sequence_capacity = 10;

    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_state_msg,
        conf);

    // Need to initialize the the published message, but only have to do this once. 
    joint_state_msg.name.size = mc.jointNames.size();
    joint_state_msg.position.size = mc.jointIds.size();
    joint_state_msg.velocity.size = mc.jointIds.size();
    joint_state_msg.effort.size = mc.jointIds.size();

    for (size_t i = 0; i < mc.jointNames.size(); i++) {
        strcpy(joint_state_msg.name.data[i].data, mc.jointNames[i].c_str());
        joint_state_msg.name.data[i].size = strlen(mc.jointNames[i].c_str());
    }

    // The subscribed message will be whatever we recieve from the ROS2 agent
    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_msg_sub,
        conf);
    // Initialize node and create executor
    rclc_node_init_default(&node, "st_3215_control", "", &support);
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 5, &allocator);

    // Initialize publishers, subscribers, services and timers
    rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states");

    rclc_subscription_init_default(
        &joint_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "desired_joint_states");

    rclc_service_init_default(
        &service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
        "set_home_position");

    // Initialize timers
 
    rclc_timer_init_default(
        &feedback_timer,
        &support,
        RCL_MS_TO_NS(FEEDBACK_TIMER_TIMEOUT),
        feedback_timer_callback_wrapper);

    // Add executors
    rclc_executor_add_service(
        &executor,
        &service,
        &request_msg,
        &response_msg,
        set_home_position_callback_wrapper);

    rclc_executor_add_subscription(
        &executor,
        &joint_state_subscriber,
        &joint_msg_sub,
        joint_state_callback_wrapper,
        ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &feedback_timer);

    syncTime();
    return true;
}

void MicroRosController::run() {
    print_msg("Running ROS2");
    delay(1000);
    rclc_executor_spin(&executor);
}

bool MicroRosController::syncTime() 
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    rmw_uros_sync_session(timeout_ms);
    if (rmw_uros_epoch_synchronized()) {
        #if (_POSIX_TIMERS > 0)
            // Get time in milliseconds or nanoseconds
            int64_t time_ns = rmw_uros_epoch_nanos();
            timespec tp;
            tp.tv_sec = time_ns / 1000000000;
            tp.tv_nsec = time_ns % 1000000000;
            clock_settime(CLOCK_REALTIME, &tp);
        #else
            unsigned long long ros_time_ms = rmw_uros_epoch_millis();
            // now we can find the difference between ROS time and uC time
            time_offset = ros_time_ms - millis();
        #endif
        return true;
    }
    return false;
}

struct timespec MicroRosController::getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS

    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}


