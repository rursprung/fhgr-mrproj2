#ifndef SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP

namespace smt {
    namespace gpio_controller {

        class GpioController {
        public:
            GpioController();
            ~GpioController();
            void applyMotorSpeed(const double vLeft, const double vRight) const;

        private:
            int pi; //< the ID returned by pigpio_start
        };

    }  // namespace gpio_controller
}  // namespace smt
#endif  // SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP
