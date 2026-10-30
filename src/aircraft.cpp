#include "aero/aircraft.hpp"
#include "aero/geo.hpp"
#include <algorithm>
#include <cmath>

namespace aero {

Aircraft::Aircraft(AircraftState s, Waypoint dest, double targetAltM, double climbRateMps)
    : state_(std::move(s)), dest_(dest), targetAltM_(targetAltM), climbRateMps_(climbRateMps) {}

void Aircraft::step(double dt) {
  if (arrived_) return;

  const double remaining = distanceM(state_.lat, state_.lon, dest_.lat, dest_.lon);
  const double travel = state_.speedMps * dt;
  if (travel >= remaining) {
    state_.lat = dest_.lat;
    state_.lon = dest_.lon;
    arrived_ = true;
    return;
  }

  state_.headingDeg = bearingDeg(state_.lat, state_.lon, dest_.lat, dest_.lon);
  destinationPoint(state_.lat, state_.lon, state_.headingDeg, travel, state_.lat, state_.lon);

  const double dAlt = targetAltM_ - state_.altM;
  const double maxDelta = climbRateMps_ * dt;
  state_.altM += std::clamp(dAlt, -maxDelta, maxDelta);
}

}  // namespace aero
