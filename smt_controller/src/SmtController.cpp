#include "SmtController.hpp"

#include <geometry_msgs/Twist.h>

namespace smt {
    namespace controller {

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
            // TODO
            /*
            const auto min = Algorithm::minimumDistance(msg.ranges, msg.range_min, msg.range_max);
            if (min.has_value()) {
                const auto angle = Algorithm::headingToNearestPoint(msg.ranges, *min, msg.angle_min, msg.angle_max,
                                                                    msg.angle_increment);

                ROS_DEBUG("smallest distance measurement: %f with angle %f", *min, angle);

                if (this->state == Started) {
                    if (*min > MIN_DISTANCE) {
                        this->publishDriveTowardsCommand(*min, angle);
                    } else {
                        this->publishDriveStopCommand();
                    }
                }
            } else {
                ROS_DEBUG("no distance measurement (nothing in sight)");

                this->publishDriveStopCommand();
            }
             */
        }

    } // smt
} // controller