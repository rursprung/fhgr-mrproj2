#include <ros/ros.h>

#include "smt_qr_detection/qr_detector.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smt_qr_detector");
    ros::NodeHandle nodeHandle("~");

    smt::qr_detector::QRDetector QRDetector(nodeHandle);

    ros::spin();
    return 0;
}
