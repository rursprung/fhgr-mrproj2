#ifndef SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
#define SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP

#include <ros/ros.h>
#include <smt_svcs/FireGun.h>

namespace smt {
    namespace gun_controller {
        class GunController {

        public:
            explicit GunController(ros::NodeHandle &nodeHandle);
            ~GunController();

        private:
            int pi; //< the ID returned by pigpio_start

            ros::ServiceServer service;

            void initPi();

            void fireShot() const;
            void moveGunToAngle(int targetAngle) const;

            bool gunCommandCallback(smt_svcs::FireGun::Request& request, smt_svcs::FireGun::Response& response);
        };

    } // namespace movement_controller
} // namespace smt

#endif // SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
