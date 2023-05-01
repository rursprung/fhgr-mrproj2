#include <ros/ros.h>

#include "smt_movement_controller/movement_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "smt_movement_controller");
    ros::NodeHandle nodeHandle("~");

    smt::movement_controller::MovementController MovementController(nodeHandle);
    ros::spin();
    return 0;
}
