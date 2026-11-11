#pragma once

#include "pros/misc.hpp"

namespace PISTON {

// Set the piston output. true extends/activates it; false retracts/deactivates it.
bool set(bool extended);

// Switch the piston from its current state to the opposite state.
bool toggle();

// Call repeatedly from driver control. Toggles once when buttonPressed changes
// from false to true, and does not repeat while the button stays held.
bool toggle(bool buttonPressed);

// Read the selected master-controller button and toggle once per press.
bool toggle(pros::controller_digital_e_t button);

// Extend while buttonHeld is true, and retract when it becomes false.
// Call this repeatedly from the driver-control loop.
bool hold(bool buttonHeld);

// Read the selected master-controller button and extend while held.
bool hold(pros::controller_digital_e_t button);

} // namespace PISTON
