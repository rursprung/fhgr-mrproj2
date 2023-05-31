#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include "smt_gpio_controller/gpio.h"

#include "smt_gun_controller/gun_controller.hpp"



namespace smt {
    namespace gun_controller {

        const int positivePipeLimitation = 50;
        const int negativePipeLimitation = -15;
        const int positivePulsewidthLimitation = 2050;
        const int negativePulsewidthLimitation = 1250;
        const int initializationPulseWidth = 1400;

        enum class Pin
        {
            GUN_TRIGGER = 18, //!< gun trigger (Pyhsical 12)
            PWM_SERVO = 12,   //!< pwm motor right (Physical 32)
        };

        /**
         * This function calculates the pulse width based on the given pipe angle, with limitations on the
         * angle range.
         *
         * @param pipeAngle The angle of a pipe in degrees.
         *
         * @return an integer value which represents the pulsewidth calculated based on the input pipe angle.
         */
        uint getPulsewidthFromPipeAngle(const int pipeAngle) {
            if (pipeAngle < negativePipeLimitation) {
                ROS_WARN("reached minimum angle limitiation (%i deg), setting angle to %i deg", negativePipeLimitation, negativePipeLimitation);
                return negativePulsewidthLimitation;
            }

            if (pipeAngle > positivePipeLimitation) {
                ROS_WARN("reached maximum angle limitiation (%i deg), setting angle to %i deg", positivePipeLimitation, positivePipeLimitation);
                return positivePulsewidthLimitation;
            }

            return (pipeAngle - negativePipeLimitation) *
                (positivePulsewidthLimitation - negativePulsewidthLimitation) /
                (positivePipeLimitation - negativePipeLimitation) +
                negativePulsewidthLimitation;
        }

        GunController::GunController(ros::NodeHandle& nodeHandle) {
            std::string gunTopic;
            int subscriberQueueSize;
            std::string gpioTopic;
            int gpioQueueSize;

            if (!nodeHandle.getParam("gun_controller_topic/topic", gunTopic)) {
                ROS_ERROR("failed to load the `gun_controller_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gun_controller_topic/queue_size", subscriberQueueSize)) {
                ROS_ERROR("failed to load the `gun_controller_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gpio_topic/topic", gpioTopic)) {
                ROS_ERROR("failed to load the `gpio_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gpio_topic/queue_size", gpioQueueSize)) {
                ROS_ERROR("failed to load the `gpio_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            gunSubscriber = nodeHandle.subscribe(gunTopic, subscriberQueueSize, &GunController::gunCommandCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", gunTopic.c_str(), subscriberQueueSize);

            gpioPublisher = nodeHandle.advertise<smt_gpio_controller::gpio>(gpioTopic, gpioQueueSize);

            initGunGPIO();
            ROS_INFO("init done");
        }

        void GunController::gunCommandCallback(const std_msgs::Int32::ConstPtr& gunAngle) const {
            const int targetPulsewidth = getPulsewidthFromPipeAngle(gunAngle->data);
            const int step = targetPulsewidth > initializationPulseWidth ? 1 : -1;
            int currentPulsewidth = initializationPulseWidth;

            // Gradually increase servo pulse widths to obtain smoother movement
            for (size_t i = 0; i < abs(targetPulsewidth - initializationPulseWidth); i++) {
                currentPulsewidth = initializationPulseWidth + i * step;
                setServoHeight(currentPulsewidth);
                ros::Duration(0.002).sleep();
            }

            // this delay before and after the shot are for a better looking
            ros::Duration(0.5).sleep();
            fireOneShot();
            ros::Duration(0.5).sleep();

            for (size_t i = abs(targetPulsewidth - initializationPulseWidth); i > 0; i--) {
                currentPulsewidth = initializationPulseWidth + i * step;
                setServoHeight(currentPulsewidth);
                ros::Duration(0.002).sleep();
            }

        }

        void GunController::initGunGPIO() const {
            setServoHeight(initializationPulseWidth);
            sendGpioCommand(static_cast<uint>(Pin::GUN_TRIGGER), "GPIO", 1);
        }

        void GunController::setServoHeight(const uint pulsewidth) const {
            sendGpioCommand(static_cast<uint>(Pin::PWM_SERVO), "SERVO", pulsewidth);
        }

        void GunController::fireOneShot() const {
            sendGpioCommand(static_cast<uint>(Pin::GUN_TRIGGER), "GPIO", 0);
            ros::Duration(0.1).sleep();
            sendGpioCommand(static_cast<uint>(Pin::GUN_TRIGGER), "GPIO", 1);
        }

        void GunController::sendGpioCommand(const uint gpio, const std::string mode, const uint value) const {
            smt_gpio_controller::gpio msg;
            msg.gpio = gpio;
            msg.mode = mode;
            msg.value = value;
            gpioPublisher.publish(msg);
        }

    } // namespace movement_controller
} // namespace smt
