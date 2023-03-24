#include <ros/ros.h>
#include <smt_movement_controller/controller.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_controller");
    ros::NodeHandle nodeHandle("~");

    smt::gpioController gpioController(nodeHandle);
    ros::spin();
    return 0;
}


