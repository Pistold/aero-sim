#include "aero/tracker.hpp"

namespace aero {

const Track& Tracker::update(const SensorReading& r) {
  auto it = tracks_.find(r.id);
  if (it == tracks_.end()) {
    Track t{r.id, r.lat, r.lon, r.altM, r.speedMps, r.t, 1};
    return tracks_.emplace(r.id, t).first->second;
  }
  Track& t = it->second;
  auto blend = [&](double old, double meas) { return old + alpha_ * (meas - old); };
  t.lat = blend(t.lat, r.lat);
  t.lon = blend(t.lon, r.lon);
  t.altM = blend(t.altM, r.altM);
  t.speedMps = blend(t.speedMps, r.speedMps);
  t.lastUpdateT = r.t;
  ++t.updates;
  return t;
}

const Track* Tracker::find(const std::string& id) const {
  auto it = tracks_.find(id);
  return it == tracks_.end() ? nullptr : &it->second;
}

}  // namespace aero
