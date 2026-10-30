#pragma once
#include "aero/sensor.hpp"
#include <map>
#include <string>

namespace aero {

struct Track {
  std::string id;
  double lat = 0, lon = 0, altM = 0, speedMps = 0;
  double lastUpdateT = 0;
  int updates = 0;
};

// Maintains one smoothed track per aircraft (simple alpha filter).
// TODO: replace with an alpha-beta or Kalman filter.
class Tracker {
 public:
  explicit Tracker(double alpha = 0.4) : alpha_(alpha) {}

  const Track& update(const SensorReading& r);
  const Track* find(const std::string& id) const;
  void remove(const std::string& id) { tracks_.erase(id); }  // e.g. after landing
  const std::map<std::string, Track>& tracks() const { return tracks_; }

 private:
  double alpha_;
  std::map<std::string, Track> tracks_;
};

}  // namespace aero
