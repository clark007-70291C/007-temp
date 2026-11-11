#pragma once

#include <cstdint>

namespace liftpid {

// Named lift heights from the original program.
enum Position {
    BASE,
    FIRST,
    SECOND,
    THIRD,
    FOURTH,
    FIFTH,
    SIXTH
};

// Start the lift controller task once from initialize().
void initialize();

// Move to a lift height in inches. This returns immediately; the PID task moves
// the lift in the background. Example: liftpid::moveTo(5.5);
// Returns false if the height is negative or not a valid number.
bool moveTo(double heightInches);

// Move to one of the named preset heights above.
bool moveTo(Position preset);

// Use manual power from -127 to 127. Nonzero power cancels a preset move.
// Zero power stops manual motion but leaves an active preset move running.
void manualControl(double power);

// Manual power control with an optional bottom-limit override.
void manualControl(double power, bool overrideBottomLimit);

// Convert the raise/lower button states into manual lift control.
void manualControl(bool raiseButton, bool lowerButton);

// Like the button overload above, but allow lowering past zero when the
// override button is held.
void manualControl(bool raiseButton, bool lowerButton, bool overrideBottomLimit);

// Stop the lift and disable automatic position control.
void stop();

// Set the lift's current position to zero. Call when the lift is at its
// chosen zero point, such as the bottom before autonomous starts.
void tarePosition();

// Read the current lift height in inches relative to the zero point.
double getPositionInches();

// Read the temperature in Celsius of one of the lift motors by group index.
double getMotorTemperature(std::uint8_t motorIndex);

} // namespace liftpid
