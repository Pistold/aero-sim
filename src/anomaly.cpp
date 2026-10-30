#include "aero/anomaly.hpp"
#include <cmath>
#include <sstream>

namespace aero {

void AnomalyDetector::raise(double t, const std::string& id, const std::string& type,
                            const std::string& msg) {
  const std::string key = id + "|" + type;
  if (active_.insert(key).second) log_.add(t, id, type, msg);
}

void AnomalyDetector::checkReading(const SensorReading& r) {
  // Speed range (stateful: logs once until it returns to normal)
  if (r.speedMps < cfg_.minSpeedMps || r.speedMps > cfg_.maxSpeedMps) {
    std::ostringstream m;
    m << "speed " << r.speedMps << " m/s outside [" << cfg_.minSpeedMps << ", "
      << cfg_.maxSpeedMps << "]";
    raise(r.t, r.id, "SPEED_RANGE", m.str());
  } else {
    clear(r.id, "SPEED_RANGE");
  }

  // Altitude discontinuity vs previous reading
  auto it = last_.find(r.id);
  if (it != last_.end()) {
    const double jump = std::fabs(r.altM - it->second.altM);
    if (jump > cfg_.maxAltJumpM) {
      std::ostringstream m;
      m << "altitude changed " << jump << " m between readings";
      log_.add(r.t, r.id, "ALT_JUMP", m.str());
    }
  }
  last_[r.id] = r;
  clear(r.id, "DATA_DROPOUT");  // fresh data received
}

void AnomalyDetector::checkStale(double t, const Tracker& tracker) {
  for (const auto& [id, trk] : tracker.tracks()) {
    if (t - trk.lastUpdateT > cfg_.dropoutSeconds) {
      std::ostringstream m;
      m << "no data for " << (t - trk.lastUpdateT) << " s";
      raise(t, id, "DATA_DROPOUT", m.str());
    }
  }
}

}  // namespace aero
