#include <pigpio.h>
#include <ros/ros.h>

#include "smt_gun_controller/gpio_controller.hpp"

enum class Pin
{
    GUN_TRIGGER = 18, //!< gun trigger (Pyhsical 12)
    PWM_SERVO = 12,   //!< pwm motor right (Physical 32)
};

bool isNodeRunning(const std::string& nodeName) {
    std::string callerId = ros::this_node::getName();
    std::string nodeNamespace = ros::names::parentNamespace(callerId);

    std::string resolvedNodeName = ros::names::resolve(nodeNamespace, nodeName);

    ros::V_string nodes;
    ros::master::getNodes(nodes);

    for (const auto& node : nodes) {
        if (node == resolvedNodeName) {
            return true;
        }
    }
    return false;
}

namespace smt {

    namespace gpio_controller {

        void initPi(const int initialPulseWidh) {
            //Check if gpio library allready initialised
            if (!isNodeRunning("/smt_movement_controller")) {
                setenv("PIGPIO_PORT", "8889", 1);
                if (gpioInitialise() < 0)
                {
                    throw std::runtime_error("error while trying to initialize pigpio!");
                    return;
                }
            }

            // PWMs
            gpioSetMode(static_cast<uint>(Pin::GUN_TRIGGER), PI_OUTPUT);
            gpioSetMode(static_cast<uint>(Pin::PWM_SERVO), PI_OUTPUT);

            gpioServo(static_cast<uint>(Pin::PWM_SERVO), initialPulseWidh);
            gpioWrite(static_cast<uint>(Pin::GUN_TRIGGER), 1);
        }

        void setServoHeight(const int pulseWidth) {
            gpioServo(static_cast<uint>(Pin::PWM_SERVO), pulseWidth);
        }

        void fireOneShot() {
            gpioWrite(static_cast<uint>(Pin::GUN_TRIGGER), 0);
            ros::Duration(0.1).sleep();
            gpioWrite(static_cast<uint>(Pin::GUN_TRIGGER), 1);
        }

    } // namespace gpio_controller

} // namespace smt
