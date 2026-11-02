#pragma once
#include "aero/kalman.hpp"
#include "aero/sensor.hpp"
#include <map>
#include <string>

namespace aero {

// Public, filtered view of one aircraft.
struct Track {
  std::string id;
  double lat = 0, lon = 0, altM = 0;
  double speedMps = 0;                       // ground speed from the filter
  double vEastMps = 0, vNorthMps = 0, climbRateMps = 0;
  double lastUpdateT = 0;
  int updates = 0;
};

struct TrackerConfig {
  // Should match the sensor's noise (the filter needs to know how much to trust it).
  double posSigmaM = 20.0;
  double altSigmaM = 10.0;
  // How much the aircraft's velocity is allowed to wander (tuning knob):
  // small = smooth but slow to follow turns, large = responsive but noisy.
  double accelSigmaMps2 = 0.2;
  double vertAccelSigmaMps2 = 0.5;
  // Initial velocity uncertainty (we know nothing about velocity at first sight).
  double initSpeedSigmaMps = 150.0;
  double initClimbSigmaMps = 20.0;
};

// Maintains one constant-velocity Kalman track per aircraft id.
// Horizontal motion is modelled in a local flat east/north frame centred on the
// first reading of each track (fine for the ~100 km scales used here).
class Tracker {
 public:
  explicit Tracker(TrackerConfig cfg = {}) : cfg_(cfg) {}

  const Track& update(const SensorReading& r);
  const Track* find(const std::string& id) const;
  const std::map<std::string, Track>& tracks() const { return tracks_; }
  void remove(const std::string& id);  // e.g. after landing

 private:
  struct Filter {
    double lat0 = 0, lon0 = 0, mPerDegLon = 1;
    KalmanAxis east, north, up;
  };

  TrackerConfig cfg_;
  std::map<std::string, Track> tracks_;
  std::map<std::string, Filter> filters_;
};

}  // namespace aero
