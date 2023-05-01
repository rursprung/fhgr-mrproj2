#include <pigpio.h>
#include <ros/ros.h>

#include "signal.h"
#include "smt_movement_controller/movement_controller.hpp"

void mySignitHandler(int sig) {
    gpioTerminate();
    ROS_INFO("mySignitHandler called!");
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "smt_movement_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");
    signal(SIGINT, mySignitHandler);

    smt::movement_controller::MovementController MovementController(nodeHandle);

    ros::spin();
    return 0;
}
