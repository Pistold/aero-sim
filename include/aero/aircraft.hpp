#pragma once
#include <string>

namespace aero {

struct Waypoint {
  double lat = 0, lon = 0;
};

struct AircraftState {
  std::string id;
  double lat = 0, lon = 0;   // degrees
  double altM = 0;           // meters
  double speedMps = 0;       // ground speed
  double headingDeg = 0;
};

// Ground-truth aircraft: flies toward a destination at constant speed
// and moves its altitude toward a target at a fixed climb rate.
class Aircraft {
 public:
  Aircraft(AircraftState s, Waypoint dest, double targetAltM, double climbRateMps = 10.0);

  void step(double dtSeconds);

  const AircraftState& state() const { return state_; }
  const Waypoint& destination() const { return dest_; }
  bool arrived() const { return arrived_; }

 private:
  AircraftState state_;
  Waypoint dest_;
  double targetAltM_, climbRateMps_;
  bool arrived_ = false;
};

}  // namespace aero
