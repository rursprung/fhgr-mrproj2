#ifndef SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
#define SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace smt {
    namespace gun_controller {
        class GunController {

        public:
            explicit GunController(ros::NodeHandle &nodeHandle);
            ~GunController();

        private:
            int pi; //< the ID returned by pigpio_start

            ros::Subscriber gunSubscriber;

            void initPi();

            void fireShot() const;
            void moveGunToAngle(int targetAngle) const;

            void gunCommandCallback(const std_msgs::Int32::ConstPtr &gunAngle) const;
        };

    } // namespace movement_controller
} // namespace smt

#endif // SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
