#ifndef SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "smt_movement_controller/gpio_controller.hpp"

namespace smt {

    namespace movement_controller {

        class MovementController {
        public:
            MovementController(ros::NodeHandle& nodeHandle);
            void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& velocity) const;
            std::pair<double, double> mapRobotVelocityToMotorSpeed(const double& linear_speed, const double& angular_speed) const;

        private:
            ros::Subscriber velSubscriber;
            gpio_controller::GpioController gpioController;
        };
    }  // namespace movement_controller

}  // namespace smt

#endif  // SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
