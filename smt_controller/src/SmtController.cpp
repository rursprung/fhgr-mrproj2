#include "SmtController.hpp"

#include <geometry_msgs/Twist.h>

namespace smt {
    namespace controller {

        const auto MIN_DISTANCE = 1.0f;

        SmtController::SmtController(const ros::NodeHandle &nodeHandle) : nodeHandle(nodeHandle) {
            std::string topic;
            if (!this->nodeHandle.getParam("scan_topic/topic", topic)) {
                ROS_ERROR("failed to load the `scan_topic/topic` parameter!");
                ros::requestShutdown();
            }
            int queue_size;
            if (!this->nodeHandle.getParam("scan_topic/queue_size", queue_size)) {
                ROS_ERROR("failed to load the `scan_topic/queue_size` parameter!");
                ros::requestShutdown();
            }
            ROS_INFO("starting subscriber for %s with queue size %i", topic.c_str(), queue_size);
            this->scanSubscriber = this->nodeHandle.subscribe(topic, queue_size, &SmtController::scanCallback, this);

            std::string cmd_vel_topic;
            if (!this->nodeHandle.getParam("cmd_vel_topic", cmd_vel_topic)) {
                ROS_ERROR("failed to load the `cmd_vel_topic` parameter!");
                ros::requestShutdown();
            }
            this->velocityCommandPublisher = this->nodeHandle.advertise<geometry_msgs::Twist>(cmd_vel_topic, 0);

            ROS_INFO("init done");
        }

        void SmtController::scanCallback(const sensor_msgs::LaserScan &msg) {
            if (!this->pathFinder.has_value()) {
                // we get the ranges only with the first message, so we have to construct it here
                ROS_INFO(
                        "received first scan message, starting path finder with measurement range [%f..%f], angles [%f..%f] (increment: %f)",
                        msg.range_min, msg.range_max, msg.angle_min, msg.angle_max, msg.angle_increment);
                this->pathFinder.emplace(msg.range_min, msg.range_max, msg.angle_min, msg.angle_max,
                                         msg.angle_increment, MIN_DISTANCE);
            }

            const auto newHeading = (*this->pathFinder).calculateNewHeading(msg.ranges);

            if (newHeading.has_value()) {
                ROS_DEBUG("new heading: %f", *newHeading);
                this->publishDriveTowardsCommand(*newHeading);
            } else {
                ROS_DEBUG("no new heading (nothing in sight)!");
                this->publishDriveStopCommand();
            }
        }

        void SmtController::publishDriveTowardsCommand(const float angle) const {
            geometry_msgs::Twist twist;
            twist.linear.x = 2;
            twist.angular.z = angle * 20;
            this->velocityCommandPublisher.publish(twist);
        }

        void SmtController::publishDriveStopCommand() const {
            const geometry_msgs::Twist twist;
            this->velocityCommandPublisher.publish(twist);
        }

    } // smt
} // controller