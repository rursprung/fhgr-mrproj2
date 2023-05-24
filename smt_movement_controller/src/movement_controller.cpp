//#include <pigpio.h>
#include <ros/ros.h>


#include "smt_movement_controller/movement_controller.hpp"
#include "smt_movement_controller/gpio.h"

enum class Pin {
    PWM_LEFT = 4,      //!< pwm motor Left (Pyhsical 7)
    PWM_RIGHT = 5,     //!< pwm motor right (Physical 29)
    DIR_LEFT_1 = 17,   //!< direction pin left 1 (Physical 11)
    DIR_LEFT_2 = 27,   //!< direction pin left 2 (Physical 13)
    DIR_RIGHT_1 = 6,   //!< direction pin right 1 (Physical 31)
    DIR_RIGHT_2 = 13,  //!< direction pin right 2 (Physical 33)
};

const int PWM_FREQ = 20000;      //!< Hertz
const int PWM_RANGE = 100;       //!< Range for pwm 0% to 100%
const double MAX_SPEED = 0.715;  //!< m/s measured


/**
 * The function maps a value from one range to another range and returns the mapped value, with
 * warnings if the input value is outside the input range.
 *
 * @param value The input value that needs to be mapped to a new range.
 * @param in_min The minimum input value that can be mapped.
 * @param in_max The maximum input value that can be mapped.
 * @param out_min The minimum value of the output range that the input value will be mapped to.
 * @param out_max The maximum output value that the function can return. It is the upper limit of the
 * range of values that the input value will be mapped to.
 *
 * @return a double value that is the result of mapping the input value from the input range (in_min to
 * in_max) to the output range (out_min to out_max). If the input value is outside the input range, the
 * function returns either the out_min or out_max value depending on which boundary is exceeded.
 */
double map(double value, double in_min, double in_max, double out_min, double out_max) {
    if (value < in_min) {
        ROS_WARN("Reached smallest possible value, return value set to out_min Value: %f", out_min);
        return out_min;
    }
    else if (value > in_max) {
        ROS_WARN("Reached highest possible value, return value set to out_max Value: %f", out_max);
        return out_max;
    }
    else {
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}


namespace smt {
    namespace movement_controller {
        MovementController::MovementController(ros::NodeHandle& nodeHandle) {
            std::string velTopic;
            int subscriberQueueSize;
            std::string gpioTopic;
            int gpioQueueSize;

            if (!nodeHandle.getParam("cmd_vel_topic/topic", velTopic)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("cmd_vel_topic/queue_size", subscriberQueueSize)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("cmd_gpio/topic", gpioTopic)) {
                ROS_ERROR("failed to load the `gpio_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gpio_topic/queue_size", gpioQueueSize)) {
                ROS_ERROR("failed to load the `gpio_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            velSubscriber = nodeHandle.subscribe(velTopic, subscriberQueueSize, &MovementController::velocityCommandCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", velTopic.c_str(), subscriberQueueSize);

            gpioPublisher = nodeHandle.advertise<smt_movement_controller::gpio>(gpioTopic, gpioQueueSize);

            initMovementGPIO();
            ROS_INFO("init done");
        }

        MovementController::~MovementController() {

        }

        void MovementController::velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& velocity) const {
            std::pair<double, double> leftAndRightSpeed = mapRobotVelocityToMotorSpeed(velocity->linear.x, velocity->angular.z);
            applyMotorSpeed(leftAndRightSpeed.first, leftAndRightSpeed.second);
        }

        void MovementController::applyMotorSpeed(const double vLeft, const double vRight) const {
            int pwm_val_left = map(abs(vLeft), 0, MAX_SPEED, 0, PWM_RANGE);
            int pwm_val_right = map(abs(vRight), 0, MAX_SPEED, 0, PWM_RANGE);

            generatePublisherMsg(static_cast<uint>(Pin::DIR_LEFT_1), "GPIO", vLeft < 0 ? 1 : 0);
            generatePublisherMsg(static_cast<uint>(Pin::DIR_LEFT_2), "GPIO", vLeft < 0 ? 1 : 0);

            generatePublisherMsg(static_cast<uint>(Pin::DIR_RIGHT_1), "GPIO", vRight < 0 ? 0 : 1);
            generatePublisherMsg(static_cast<uint>(Pin::DIR_RIGHT_2), "GPIO", vRight < 0 ? 0 : 1);

            generatePublisherMsg(static_cast<uint>(Pin::PWM_LEFT), "PWM", pwm_val_left);
            generatePublisherMsg(static_cast<uint>(Pin::PWM_RIGHT), "PWM", pwm_val_right);
        }

        void MovementController::generatePublisherMsg(const uint gpio, const std::string mode, const int value) const {
            smt_movement_controller::gpio msg;
            msg.gpio = gpio;
            msg.mode = mode;
            msg.value = value;
            gpioPublisher.publish(msg);
        }

        std::pair<double, double> MovementController::mapRobotVelocityToMotorSpeed(const double& linear_speed, const double& angular_speed) const {
            double wheelSeperation = 0.201;  // Wheelseperation in m
            double vel_mot_left = linear_speed - (angular_speed * wheelSeperation) / 2;
            double vel_mot_right = linear_speed + (angular_speed * wheelSeperation) / 2;
            return { vel_mot_left, vel_mot_right };
        }

        void MovementController::initMovementGPIO() {
            const double vLeftInitial = 0.0;
            const double vRightInitial = 0.0;
            applyMotorSpeed(vLeftInitial, vRightInitial);
        }

    }  // namespace movement_controller
}  // namespace smt
