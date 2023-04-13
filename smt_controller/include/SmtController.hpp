#ifndef SMT_CONTROLLER_SMTCONTROLLER_HPP
#define SMT_CONTROLLER_SMTCONTROLLER_HPP

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
        };

    } // smt
} // controller

#endif //SMT_CONTROLLER_SMTCONTROLLER_HPP
