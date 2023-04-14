#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <smt_camera_controller/smt_camera_controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smt_camera_publisher");
    ros::NodeHandle nodeHandle("~");

    smt::cameraController camera =  smt::cameraController(nodeHandle);

    ros::Rate loop_rate(30);
    while (nodeHandle.ok())
    {
        camera.updateImg();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
