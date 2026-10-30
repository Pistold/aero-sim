#pragma once
#include "aero/event_log.hpp"
#include "aero/sensor.hpp"
#include "aero/tracker.hpp"
#include <map>
#include <set>
#include <string>

namespace aero {

struct AnomalyConfig {
  double minSpeedMps = 60.0;
  double maxSpeedMps = 320.0;
  double maxAltJumpM = 500.0;     // between consecutive readings
  double dropoutSeconds = 5.0;    // no data for this long => dropout
};

class AnomalyDetector {
 public:
  AnomalyDetector(EventLog& log, AnomalyConfig cfg = {}) : log_(log), cfg_(cfg) {}

  // Check a raw reading against the previous one for that aircraft.
  void checkReading(const SensorReading& r);
  // Check all tracks for stale data at time t.
  void checkStale(double t, const Tracker& tracker);

 private:
  EventLog& log_;
  AnomalyConfig cfg_;
  std::map<std::string, SensorReading> last_;
  std::set<std::string> active_;  // "id|TYPE" currently flagged (avoids log spam)

  void raise(double t, const std::string& id, const std::string& type, const std::string& msg);
  void clear(const std::string& id, const std::string& type) { active_.erase(id + "|" + type); }
};

}  // namespace aero
