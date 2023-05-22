#include <ros/ros.h>

#include "smt_gun_controller/gun_controller.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_gun_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    smt::gun_controller::GunController GunController(nodeHandle);

    ros::spin();
    return 0;
}
