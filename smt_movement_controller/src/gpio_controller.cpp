#include <pigpiod_if2.h>
#include "smt_movement_controller/gpio_controller.hpp"
#include "smt_movement_controller/movement_controller.hpp"

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

    namespace gpio_controller {

        GpioController::GpioController() {
            this->pi = pigpio_start(NULL, NULL);
            if (this->pi < 0) {
                throw std::runtime_error("error while trying to initialize pigpio!");
                return;
            }

            // PWMs
            set_mode(this->pi, static_cast<uint>(Pin::PWM_LEFT), PI_OUTPUT);
            set_mode(this->pi, static_cast<uint>(Pin::PWM_RIGHT), PI_OUTPUT);

            set_PWM_frequency(this->pi, static_cast<uint>(Pin::PWM_LEFT), PWM_FREQ);
            set_PWM_frequency(this->pi, static_cast<uint>(Pin::PWM_RIGHT), PWM_FREQ);

            set_PWM_range(this->pi, static_cast<uint>(Pin::PWM_LEFT), PWM_RANGE);
            set_PWM_range(this->pi, static_cast<uint>(Pin::PWM_RIGHT), PWM_RANGE);

            // GPIOs
            set_mode(this->pi, static_cast<uint>(Pin::DIR_LEFT_1), PI_OUTPUT);
            set_mode(this->pi, static_cast<uint>(Pin::DIR_LEFT_2), PI_OUTPUT);
            set_mode(this->pi, static_cast<uint>(Pin::DIR_RIGHT_1), PI_OUTPUT);
            set_mode(this->pi, static_cast<uint>(Pin::DIR_RIGHT_2), PI_OUTPUT);

            gpio_write(this->pi, static_cast<uint>(Pin::DIR_LEFT_1), 0);
            gpio_write(this->pi, static_cast<uint>(Pin::DIR_LEFT_2), 0);
            gpio_write(this->pi, static_cast<uint>(Pin::DIR_RIGHT_1), 0);
            gpio_write(this->pi, static_cast<uint>(Pin::DIR_RIGHT_2), 0);
        }

        GpioController::~GpioController() {
            pigpio_stop(this->pi);
        }

        void GpioController::applyMotorSpeed(const double vLeft, const double vRight) const {
            int pwm_val_left = map(abs(vLeft), 0, MAX_SPEED, 0, PWM_RANGE);
            int pwm_val_right = map(abs(vRight), 0, MAX_SPEED, 0, PWM_RANGE);

            gpio_write(this->pi, static_cast<uint>(Pin::DIR_LEFT_1), vLeft < 0 ? 1 : 0);
            gpio_write(this->pi, static_cast<uint>(Pin::DIR_LEFT_2), vLeft > 0 ? 1 : 0);

            gpio_write(this->pi, static_cast<uint>(Pin::DIR_RIGHT_1), vRight < 0 ? 0 : 1);
            gpio_write(this->pi, static_cast<uint>(Pin::DIR_RIGHT_2), vRight > 0 ? 0 : 1);

            set_PWM_dutycycle(this->pi, static_cast<uint>(Pin::PWM_LEFT), pwm_val_left);
            set_PWM_dutycycle(this->pi, static_cast<uint>(Pin::PWM_RIGHT), pwm_val_right);
        }

    }  // namespace gpio_controller

}  // namespace smt
