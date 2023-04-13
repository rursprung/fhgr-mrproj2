#ifndef SMT_CONTROLLER_SMTCONTROLLER_HPP
#define SMT_CONTROLLER_SMTCONTROLLER_HPP

#include "PathFinder.hpp"

#include <tl/optional.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace smt {
    namespace controller {

        class SmtController {
        public:
            explicit SmtController(const ros::NodeHandle &nodeHandle);

            void scanCallback(const sensor_msgs::LaserScan &msg);
        private:
            ros::NodeHandle nodeHandle;
            ros::Subscriber scanSubscriber;
            ros::Publisher velocityCommandPublisher;

            tl::optional<algorithm::PathFinder> pathFinder = tl::nullopt;

            void publishDriveTowardsCommand(float angle) const;
            void publishDriveStopCommand() const;
        };

    } // smt
} // controller

#endif //SMT_CONTROLLER_SMTCONTROLLER_HPP
