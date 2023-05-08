#ifndef SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP
#define SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP

namespace smt {

namespace gpio_controller {

void initPi();
void applyMotorSpeed(const double vLeft, const double vRight);

}  // namespace gpio_controller

}  // namespace smt

#endif  // SMT_MOVEMENT_CONTROLLER_GPIOCONTROLLER_HPP
