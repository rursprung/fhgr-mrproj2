#include <ros/ros.h>

#include "smt_qr_controller/smt_qr_controller.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_qr_controller");
    ros::NodeHandle nodeHandle("~");

    smt::qr_controller::QrController qrController(nodeHandle);

    ros::spin();
    return 0;
}
