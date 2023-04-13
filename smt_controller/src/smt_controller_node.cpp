#include <ros/ros.h>

#include "SmtController.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "smb_highlevel_controller");
    const ros::NodeHandle nodeHandle("~");

    const smt::controller::SmtController controller(nodeHandle);

    ros::spin();

    return 0;
}
