#include <pigpio.h>
#include <ros/ros.h>

#include "smt_gpio_controller/gpio_controller.hpp"
#include "smt_gpio_controller/gpio.h"

const int PWM_FREQ = 20000;      //!< Hertz
const int PWM_RANGE = 100;       //!< Range for pwm 0% to 100%

namespace smt {
    namespace gpio_controller {
        GpioController::GpioController(ros::NodeHandle& nodeHandle) {
            std::string gpioTopic;
            int gpioQueueSize;

            initPi();

            if (!nodeHandle.getParam("cmd_gpio/topic", gpioTopic)) {
                ROS_ERROR("failed to load the `gpio_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gpio_topic/queue_size", gpioQueueSize)) {
                ROS_ERROR("failed to load the `gpio_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            gpioSubscriber = nodeHandle.subscribe(gpioTopic, gpioQueueSize, &GpioController::gpioCommandCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", gpioTopic.c_str(), gpioQueueSize);

            ROS_INFO("RPi init done");
        }
        GpioController::~GpioController() {
            gpioTerminate();
        }

        void GpioController::initPi() const {
            setenv("PIGPIO_PORT", "8889", 1);
            if (gpioInitialise() < 0)
            {
                throw std::runtime_error("error while trying to initialize pigpio!");
                return;
            }
        }

        void GpioController::gpioCommandCallback(const smt_gpio_controller::gpio& gpioSettings) const {
            uint gpio = gpioSettings.gpio;
            std::string mode = gpioSettings.mode;
            int value = gpioSettings.value;

            if (gpio > 53) {
                ROS_WARN("invalid GPIO, only 0 - 53 permitted, nothing set!");
                return;
            }

            //if GPIO mode is not set, set the mode
            if (gpioGetMode(gpio) == PI_BAD_GPIO) {
                gpioSetMode(gpio, PI_OUTPUT);
                if (mode == "PWM") {
                    gpioSetPWMfrequency(gpio, PWM_FREQ);
                    gpioSetPWMrange(gpio, PWM_RANGE);
                }
            }

            if (mode == "GPIO") {
                if (value > 1) {
                    ROS_WARN("Bad GPIO Value, only 1 or 0 permitted, GPIO: %i not set!", gpio);
                    return;
                }
                gpioWrite(gpio, value);
            }
            else if (mode == "PWM") {
                if (value > 100) {
                    ROS_WARN("Bad GPIO Value, only 0 to 100 permitted, GPIO: %i not set!", gpio);
                    return;
                }
                gpioPWM(gpio, value);
            }
            else if (mode == "SERVO") {
                if (value < 500 or value > 2500) {
                    ROS_WARN("Bad GPIO Value, only 500 to 2500 permitted, GPIO: %i not set!", gpio);
                    return;
                }
                gpioServo(gpio, value);
            }
        }

    } // namespace gpio_controller
} // namespace smt
