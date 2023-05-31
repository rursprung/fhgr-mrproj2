#include <pigpio.h>
#include <ros/ros.h>

#include "smt_gpio_controller/gpio_controller.hpp"

const int PWM_FREQ = 20000;      //!< Hertz
const int PWM_RANGE = 100;       //!< Range for pwm 0% to 100%

namespace smt {
    namespace gpio_controller {
        GpioController::GpioController(ros::NodeHandle& nodeHandle) {
            std::string gpioTopic;
            int gpioQueueSize;

            initPi();

            if (!nodeHandle.getParam("gpio_topic/topic", gpioTopic)) {
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
            auto const& gpio = gpioSettings.gpio;
            auto const& mode = gpioSettings.mode;
            auto const& value = gpioSettings.value;

            if (gpio > 53) {
                ROS_ERROR("invalid GPIO, only 0 - 53 permitted, nothing set!");
                return;
            }

            //if GPIO mode is not set, set the mode
            if (gpioGetMode(gpio) == PI_BAD_GPIO) {
                gpioSetMode(gpio, PI_OUTPUT);

            }

            if (mode == "PWM") {
                handlePwmUpdate(gpio, value);
            }
            else if (mode == "SERVO") {
                handleServoUpdate(gpio, value);
            }
            else if (mode == "GPIO") {
                handleGpioUpdate(gpio, value);
            }
            else {
                ROS_ERROR("Bad GPIO Mode, only PWM, SERVO, GPIO permitted, GPIO: %i not set!", gpio);
            }

        }

        void GpioController::handlePwmUpdate(const uint8_t gpio, const uint32_t value) const {
            if (value > 100) {
                ROS_ERROR("Bad GPIO PWM-Value, only 0 to 100 permitted, GPIO: %i not set!", gpio);
                return;
            }
            if (gpioGetPWMrange(gpio) != PWM_RANGE) {
                gpioSetPWMfrequency(gpio, PWM_FREQ);
                gpioSetPWMrange(gpio, PWM_RANGE);
            }

            gpioPWM(gpio, value);
        }

        void GpioController::handleServoUpdate(const uint8_t gpio, const uint32_t value) const {
            if (value < 500 or value > 2500) {
                ROS_ERROR("Bad GPIO Servo-Value, only 500 to 2500 permitted, GPIO: %i not set!", gpio);
                return;
            }
            gpioServo(gpio, value);
        }

        void GpioController::handleGpioUpdate(const uint8_t gpio, const uint32_t value) const {
            if (value > 1) {
                ROS_ERROR("Bad GPIO Value, only 1 or 0 permitted, GPIO: %i not set!", gpio);
                return;
            }
            gpioWrite(gpio, value);
        }

    } // namespace gpio_controller
} // namespace smt
