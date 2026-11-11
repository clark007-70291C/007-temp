#pragma once

#include <cstdint>

namespace brain_screen {

enum class AutonRoutine : std::uint8_t {
    CLOSE,
    FAR,
    SKILLS
};

void initialize();
void requestAutonScreenTransition();
AutonRoutine getSelectedAutonRoutine();

} // namespace brain_screen