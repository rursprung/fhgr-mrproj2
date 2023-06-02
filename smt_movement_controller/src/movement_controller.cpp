#include <pigpio.h>
#include <ros/ros.h>

#include "smt_movement_controller/movement_controller.hpp"
#include "smt_movement_controller/gpio_controller.hpp"

namespace smt {
    namespace movement_controller {
        MovementController::MovementController(ros::NodeHandle& nodeHandle) {
            std::string velTopic;
            int subscriberQueueSize;
            if (!nodeHandle.getParam("cmd_vel_topic/topic", velTopic)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("cmd_vel_topic/queue_size", subscriberQueueSize)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            velSubscriber = nodeHandle.subscribe(velTopic, subscriberQueueSize, &MovementController::velocityCommandCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", velTopic.c_str(), subscriberQueueSize);

            ROS_INFO("init done");
        }

        void MovementController::velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& velocity) const {
            std::pair<double, double> leftAndRightSpeed = mapRobotVelocityToMotorSpeed(velocity->linear.x, velocity->angular.z);
            this->gpioController.applyMotorSpeed(leftAndRightSpeed.first, leftAndRightSpeed.second);
        }

        std::pair<double, double> MovementController::mapRobotVelocityToMotorSpeed(const double& linear_speed, const double& angular_speed) const {
            double wheelSeperation = 0.201;  // Wheelseperation in m
            double vel_mot_left = linear_speed - (angular_speed * wheelSeperation) / 2;
            double vel_mot_right = linear_speed + (angular_speed * wheelSeperation) / 2;
            return { vel_mot_left, vel_mot_right };
        }

    }  // namespace movement_controller
}  // namespace smt
