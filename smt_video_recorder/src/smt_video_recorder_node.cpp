#include <ros/ros.h>
#include <smt_video_recorder/video_recorder.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_recorder");
    ros::NodeHandle nodeHandle("~");

    smt::videoController videoController(nodeHandle);
    ros::spin();
    return 0;
}
