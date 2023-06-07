#include <ros/ros.h>

#include "smt_qr_finder/QRFinder.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_qr_finder");
    ros::NodeHandle nodeHandle("~");

    smt::qr_finder::QRFinder qrFinder(nodeHandle);

    ros::spin();
    return 0;
}
