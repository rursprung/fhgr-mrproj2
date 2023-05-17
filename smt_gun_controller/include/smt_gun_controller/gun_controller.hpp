#ifndef SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
#define SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace smt
{

    namespace gun_controller
    {

        class GunController
        {
        public:
            GunController(ros::NodeHandle &nodeHandle_);
            ~GunController();
            void gunCommandCallback(const std_msgs::Int32::ConstPtr &gunAngle);

        private:
            ros::Subscriber gunSubscriber;
        };
    } // namespace movement_controller

} // namespace smt

#endif // SMT_GUN_CONTROLLER_GUN_CONTROLLER_HPP
