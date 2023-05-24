#ifndef SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP
#define SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP

#include <ros/ros.h>

#include "smt_gpio_controller/gpio.h"

void initPiGunGpio(const int initialPulseWidh);
void setServoHeight(const int pulseWidth);
void fireOneShot();

namespace smt {

    namespace gpio_controller {

        class GpioController {
        public:
            GpioController(ros::NodeHandle& nodeHandle);
            ~GpioController();
            void initPi() const;
            void gpioCommandCallback(const smt_gpio_controller::gpio& gpioSettings) const;

        private:
            ros::Subscriber gpioSubscriber;
        };


    } // namespace gpio_controller

} // namespace smt

#endif // SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP
