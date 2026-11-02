#include "aero/tracker.hpp"
#include "aero/geo.hpp"
#include <cmath>

namespace aero {

namespace {
constexpr double kMPerDegLat = kPi * kEarthRadiusM / 180.0;
}

const Track& Tracker::update(const SensorReading& r) {
  const double posVar = cfg_.posSigmaM * cfg_.posSigmaM;
  const double altVar = cfg_.altSigmaM * cfg_.altSigmaM;

  Track& trk = tracks_[r.id];
  auto it = filters_.find(r.id);

  if (it == filters_.end()) {
    // First sighting: start the filter at the measurement, velocity unknown.
    Filter f;
    f.lat0 = r.lat;
    f.lon0 = r.lon;
    f.mPerDegLon = kMPerDegLat * std::cos(deg2rad(r.lat));
    const double vVar = cfg_.initSpeedSigmaMps * cfg_.initSpeedSigmaMps;
    f.east.init(0.0, posVar, vVar);
    f.north.init(0.0, posVar, vVar);
    f.up.init(r.altM, altVar, cfg_.initClimbSigmaMps * cfg_.initClimbSigmaMps);
    it = filters_.emplace(r.id, f).first;
    trk.id = r.id;
    trk.updates = 0;
  } else {
    Filter& f = it->second;
    const double dt = r.t - trk.lastUpdateT;
    if (dt > 0.0) {  // predict across the gap (works for missed readings too)
      f.east.predict(dt, cfg_.accelSigmaMps2);
      f.north.predict(dt, cfg_.accelSigmaMps2);
      f.up.predict(dt, cfg_.vertAccelSigmaMps2);
    }
    f.east.update((r.lon - f.lon0) * f.mPerDegLon, posVar);
    f.north.update((r.lat - f.lat0) * kMPerDegLat, posVar);
    f.up.update(r.altM, altVar);
  }

  const Filter& f = it->second;
  trk.lat = f.lat0 + f.north.pos() / kMPerDegLat;
  trk.lon = f.lon0 + f.east.pos() / f.mPerDegLon;
  trk.altM = f.up.pos();
  trk.vEastMps = f.east.vel();
  trk.vNorthMps = f.north.vel();
  trk.climbRateMps = f.up.vel();
  trk.speedMps = std::hypot(trk.vEastMps, trk.vNorthMps);
  trk.lastUpdateT = r.t;
  ++trk.updates;
  return trk;
}

const Track* Tracker::find(const std::string& id) const {
  auto it = tracks_.find(id);
  return it == tracks_.end() ? nullptr : &it->second;
}

void Tracker::remove(const std::string& id) {
  tracks_.erase(id);
  filters_.erase(id);
}

}  // namespace aero
