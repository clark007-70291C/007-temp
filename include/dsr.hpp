#pragma once

namespace dsr {

/**
 * Correct robot position using the sensors and wall settings configured at
 * the top of dsr.cpp. Set unused sensor pointers there to nullptr. If both
 * directions on one axis are supplied and valid, their readings are averaged.
 * The current heading is kept.
 *
 * Returns true if at least one valid reading updated the pose. Returns false
 * without changing the pose if no provided sensor has a usable reading.
 */
bool resetWall();

} // namespace dsr
