#ifndef controller_HPP
#define controller_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void PiInit();
void applyMotorSpeed(const double V_Left, const double V_Right);

namespace smt{

    class gpioController{
        public: 
            gpioController(ros::NodeHandle &nodeHandle);
            ~gpioController();
            void scanCallback(const geometry_msgs::Twist::ConstPtr& velocity);
            void vel2motors(const double &linear, const double &angular);

            void PiInit();
            void applyMotorSpeed(const double vLeft, const double vRight);

        private:
            ros::NodeHandle nodeHandle_;
            ros::Subscriber velSubscriber_;
            int subscriberQueueSize_;
            std::string velTopic_;
    };
}

#endif // controller_HPP
