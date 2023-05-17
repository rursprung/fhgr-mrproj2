#ifndef SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace smt {

    namespace movement_controller {

        class MovementController {
        public:
            MovementController(ros::NodeHandle& nodeHandle_);
            ~MovementController();
            void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& velocity) const;
            std::pair<double, double> mapRobotVelocityToMotorSpeed(const double& linear_speed, const double& angular_speed) const;

        private:
            ros::Subscriber velSubscriber;

        };
    }  // namespace movement_controller

}  // namespace smt

#endif  // SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
