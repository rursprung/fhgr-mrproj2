#include <ros/ros.h>

#include "smt_gpio_controller/gpio_controller.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_gpio_controller");
    ros::NodeHandle nodeHandle("~");

    smt::gpio_controller::GpioController GpioController(nodeHandle);

    ros::spin();
    return 0;
}
