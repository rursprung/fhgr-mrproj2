#include <pigpio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "smt_gun_controller/gun_controller.hpp"
#include "smt_gun_controller/gpio_controller.hpp"

const int positivePipeLimitation = 50;
const int negativePipeLimitation = -15;
const int positivePulsewidthLimitation = 2050;
const int negativePulsewidthLimitation = 1250;
const int initializationPulseWidth = 1400;

/**
 * This function calculates the pulse width based on the given pipe angle, with limitations on the
 * angle range.
 *
 * @param pipeAngle The angle of a pipe in degrees.
 *
 * @return an integer value which represents the pulsewidth calculated based on the input pipe angle.
 */
int getPulsewidthFromPipeAngle(const int pipeAngle) {
    if (pipeAngle < negativePipeLimitation) {
        ROS_WARN("reached minimum angle limitiation (-15 deg), setting angle to -15 deg");
        return negativePulsewidthLimitation;
    }

    if (pipeAngle > positivePipeLimitation) {
        ROS_WARN("reached maximum angle limitiation (50 deg), setting angle to 50 deg");
        return positivePulsewidthLimitation;
    }

    return (pipeAngle - negativePipeLimitation) *
        (positivePulsewidthLimitation - negativePulsewidthLimitation) /
        (positivePipeLimitation - negativePipeLimitation) +
        negativePulsewidthLimitation;
}

namespace smt {

    namespace gun_controller {

        GunController::GunController(ros::NodeHandle& nodeHandle) {
            std::string gunTopic;
            int subscriberQueueSize;
            if (!nodeHandle.getParam("gun_controller_topic/topic", gunTopic)) {
                ROS_ERROR("failed to load the `gun_controller_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gun_controller_topic/queue_size", subscriberQueueSize)) {
                ROS_ERROR("failed to load the `gun_controller_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            gunSubscriber = nodeHandle.subscribe(gunTopic, subscriberQueueSize, &GunController::gunCommandCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", gunTopic.c_str(), subscriberQueueSize);

            smt::gpio_controller::initPi(initializationPulseWidth);
            ROS_INFO("init done");
        }

        GunController::~GunController() {
            gpioTerminate();
        }

        void GunController::gunCommandCallback(const std_msgs::Int32::ConstPtr& gunAngle) {
            int targetPulsewidth = getPulsewidthFromPipeAngle(gunAngle->data);
            int step = targetPulsewidth > initializationPulseWidth ? 1 : -1;
            int currentPulsewidth = initializationPulseWidth;

            // Gradually increase servo pulse widths to obtain smoother movement
            for (size_t i = 0; i < abs(targetPulsewidth - initializationPulseWidth); i++) {
                currentPulsewidth = initializationPulseWidth + i * step;
                smt::gpio_controller::setServoHeight(currentPulsewidth);
                ros::Duration(0.002).sleep();
            }

            ros::Duration(0.5).sleep();
            smt::gpio_controller::fireOneShot();
            ros::Duration(0.5).sleep();

            for (size_t i = abs(targetPulsewidth - initializationPulseWidth); i > 0; i--) {
                currentPulsewidth = initializationPulseWidth + i * step;
                smt::gpio_controller::setServoHeight(currentPulsewidth);
                ros::Duration(0.002).sleep();
            }

        }

    } // namespace movement_controller

} // namespace smt
