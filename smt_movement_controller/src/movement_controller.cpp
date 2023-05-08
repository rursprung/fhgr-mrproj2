#include "smt_movement_controller/movement_controller.hpp"

#include <pigpio.h>
#include <ros/ros.h>

#include "smt_movement_controller/gpio_controller.hpp"

namespace smt {
namespace movement_controller {
MovementController::MovementController(ros::NodeHandle &nodeHandle) {
    ros::NodeHandle nodeHandle_ = nodeHandle;
    if (!nodeHandle_.getParam("cmd_vel_topic/topic", velTopic_)) {
        ROS_ERROR("failed to load the `cmd_vel_topic/topic` parameter!");
        ros::requestShutdown();
    }

    if (!nodeHandle_.getParam("cmd_vel_topic/queue_size", subscriberQueueSize_)) {
        ROS_ERROR("failed to load the `cmd_vel_topic/queue_size` parameter!");
        ros::requestShutdown();
    }

    velSubscriber_ = nodeHandle_.subscribe(velTopic_, subscriberQueueSize_, &MovementController::velocityCommandCallback, this);
    ROS_INFO("starting subscriber for %s with queue size %i", velTopic_.c_str(), subscriberQueueSize_);

    smt::gpio_controller::initPi();
    ROS_INFO("init done");
}

MovementController::~MovementController() {
    gpioTerminate();
}

void MovementController::velocityCommandCallback(const geometry_msgs::Twist::ConstPtr &velocity) const {
    std::pair<double, double> leftAndRightSpeed = mapRobotVelocityToMotorSpeed(velocity->linear.x, velocity->angular.z);
    smt::gpio_controller::applyMotorSpeed(leftAndRightSpeed.first, leftAndRightSpeed.second);
}

std::pair<double, double> MovementController::mapRobotVelocityToMotorSpeed(const double &linear_speed, const double &angular_speed) const {
    double wheelSeperation = 0.201;  // Wheelseperation in m
    double vel_mot_left = linear_speed - (angular_speed * wheelSeperation) / 2;
    double vel_mot_right = linear_speed + (angular_speed * wheelSeperation) / 2;
    return std::pair<double, double>(vel_mot_left, vel_mot_right);
}

}  // namespace movement_controller
}  // namespace smt
