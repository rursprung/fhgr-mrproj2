#ifndef SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
#define SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace smt {
    namespace gun_controller {
        class GunController {

        public:
            GunController(ros::NodeHandle& nodeHandle);
            ~GunController();
            void gunCommandCallback(const std_msgs::Int32::ConstPtr& gunAngle) const;
            void initGunGPIO() const;
            void setServoHeight(const int pulsewidth) const;
            void fireOneShot() const;
            void generatePublisherMsg(const uint gpio, const std::string mode, const int value) const;

        private:
            ros::Subscriber gunSubscriber;
            ros::Publisher gpioPublisher;
        };

    } // namespace movement_controller
} // namespace smt

#endif // SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
