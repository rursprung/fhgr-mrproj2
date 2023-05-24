#ifndef SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace smt {
    namespace movement_controller {
        class MovementController {
        public:
            MovementController(ros::NodeHandle& nodeHandle);
            ~MovementController();
            void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& velocity) const;
            std::pair<double, double> mapRobotVelocityToMotorSpeed(const double& linear_speed, const double& angular_speed) const;
            void applyMotorSpeed(const double vLeft, const double vRight) const;
            void generatePublisherMsg(const uint gpio, const std::string mode, const int value) const;
            void initMovementGPIO();

        private:
            ros::Subscriber velSubscriber;
            ros::Publisher gpioPublisher;

        };
    }  // namespace movement_controller
}  // namespace smt

#endif  // SMT_MOVEMENT_CONTROLLER_MOVEMENTCONTROLLER_HPP
