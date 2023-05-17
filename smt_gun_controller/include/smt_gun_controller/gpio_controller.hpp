#ifndef SMT_GUN_CONTROLLER_GPIOCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP

namespace smt
{
    namespace gpio_controller
    {
        void initPi(const int initialPulseWidh);
        void setServoHeight(const int pulseWidth);
        void fireOneShot();
    } // namespace gpio_controller
} // namespace smt

#endif // SMT_GUN_CONTROLLER_GPIOCONTROLLER_HPP
