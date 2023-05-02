
#include <ros/ros.h>

#include "smt_camera_controller/camera_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "smt_camera_controller");
    ros::NodeHandle nodeHandle("~");

    smt::camera_controller::CameraController cameraController(nodeHandle);

    cameraController.spin();
    return 0;
}
