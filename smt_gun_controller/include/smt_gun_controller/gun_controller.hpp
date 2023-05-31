#ifndef SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
#define SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace smt {
    namespace gun_controller {
        class GunController {

        public:
            GunController(ros::NodeHandle& nodeHandle);

        private:
            ros::Subscriber gunSubscriber;
            ros::Publisher gpioPublisher;

            void initGunGPIO() const;
            void setServoHeight(const uint pulsewidth) const;
            void fireOneShot() const;
            void gunCommandCallback(const std_msgs::Int32::ConstPtr& gunAngle) const;
            void sendGpioCommand(const uint gpio, const std::string mode, const uint value) const;
        };

    } // namespace movement_controller
} // namespace smt

#endif // SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
