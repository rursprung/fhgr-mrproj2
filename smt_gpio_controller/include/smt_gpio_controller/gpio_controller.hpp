#ifndef SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP
#define SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP

#include <ros/ros.h>

#include "smt_gpio_controller/gpio.h"

namespace smt {

    namespace gpio_controller {

        class GpioController {
        public:
            GpioController(ros::NodeHandle& nodeHandle);
            ~GpioController();

        private:
            ros::Subscriber gpioSubscriber;

            void initPi() const;
            void gpioCommandCallback(const smt_gpio_controller::gpio& gpioSettings) const;
            void handlePwmUpdate(const uint8_t gpio, const uint32_t value) const;
            void handleServoUpdate(const uint8_t gpio, const uint32_t value) const;
            void handleGpioUpdate(const uint8_t gpio, const uint32_t value) const;

        };

    } // namespace gpio_controller

} // namespace smt

#endif // SMT_GPIO_CONTROLLER_GPIO_CONTROLLER_HPP
