#include "piston.hpp"

#include "pros/adi.hpp"

#include <atomic>
#include <cstdint>

extern pros::adi::DigitalOut Claw;
extern pros::Controller controller;

namespace PISTON {
namespace {

std::atomic<bool> isExtended{false};
bool toggleButtonWasPressed = false;
bool buttonWasPressed[13]{};

} // namespace

bool set(bool extended) {
    const std::int32_t result = Claw.set_value(extended);
    if (result == PROS_ERR) {
        return false;
    }

    isExtended.store(extended);
    return true;
}

bool toggle() {
    return set(!isExtended.load());
}

bool toggle(bool buttonPressed) {
    const bool pressedThisLoop = buttonPressed && !toggleButtonWasPressed;
    toggleButtonWasPressed = buttonPressed;
    return !pressedThisLoop || toggle();
}

bool toggle(pros::controller_digital_e_t button) {
    const int buttonIndex = static_cast<int>(button) -
        static_cast<int>(pros::E_CONTROLLER_DIGITAL_L1);
    if (buttonIndex < 0 || buttonIndex >= 13) {
        return false;
    }

    const bool buttonPressed = controller.get_digital(button);
    const bool pressedThisLoop = buttonPressed && !buttonWasPressed[buttonIndex];
    buttonWasPressed[buttonIndex] = buttonPressed;
    return !pressedThisLoop || toggle();
}

bool hold(bool buttonHeld) {
    return set(buttonHeld);
}

bool hold(pros::controller_digital_e_t button) {
    return hold(controller.get_digital(button));
}

} // namespace PISTON
