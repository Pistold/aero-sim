#include "aero/aircraft.hpp"
#include "aero/anomaly.hpp"
#include "aero/geo.hpp"
#include "aero/sensor.hpp"
#include "aero/tracker.hpp"
#include <fstream>
#include <iostream>
#include <vector>

using namespace aero;

int main() {
  const double dt = 1.0, duration = 900.0;

  std::vector<Aircraft> fleet;
  fleet.emplace_back(AircraftState{"AC1", 42.00, -72.00, 3000, 200, 0}, Waypoint{42.36, -71.01}, 9000);
  fleet.emplace_back(AircraftState{"AC2", 41.50, -70.50, 8000, 240, 0}, Waypoint{42.36, -71.01}, 8000);
  fleet.emplace_back(AircraftState{"AC3", 43.00, -71.50, 6000, 150, 0}, Waypoint{42.36, -71.01}, 2000);
  fleet.emplace_back(AircraftState{"AC4", 42.70, -70.60, 10000, 300, 0}, Waypoint{42.36, -71.01}, 10000);

  Sensor sensor(/*seed=*/42);
  Tracker tracker;
  EventLog log;
  AnomalyDetector detector(log);

  std::ofstream tel("telemetry.csv");
  tel << "t,id,true_lat,true_lon,true_alt,meas_lat,meas_lon,meas_alt,meas_speed,"
         "trk_lat,trk_lon,trk_alt,dist_to_dest_m,heading_deg,eta_s\n";

  for (double t = 0; t <= duration; t += dt) {
    for (auto& ac : fleet) {
      const bool wasArrived = ac.arrived();
      ac.step(dt);
      if (ac.arrived() && !wasArrived) {
        log.add(t, ac.state().id, "ARRIVED", "reached destination");
        tracker.remove(ac.state().id);  // landed: stop expecting data
        continue;
      }
      if (wasArrived) continue;

      auto reading = sensor.observe(t, ac.state());
      if (!reading) continue;  // dropout

      // Fault injection: altitude spike on AC2 and overspeed on AC4
      if (reading->id == "AC2" && t >= 400 && t < 402) reading->altM += 2000;
      if (reading->id == "AC4" && t >= 100 && t < 120) reading->speedMps = 400;

      detector.checkReading(*reading);
      const Track& trk = tracker.update(*reading);

      const auto& d = ac.destination();
      const double dist = distanceM(trk.lat, trk.lon, d.lat, d.lon);
      const double hdg = bearingDeg(trk.lat, trk.lon, d.lat, d.lon);
      const double eta = etaSeconds(dist, trk.speedMps);
      const auto& s = ac.state();
      tel << t << ',' << s.id << ',' << s.lat << ',' << s.lon << ',' << s.altM << ','
          << reading->lat << ',' << reading->lon << ',' << reading->altM << ','
          << reading->speedMps << ',' << trk.lat << ',' << trk.lon << ',' << trk.altM << ','
          << dist << ',' << hdg << ',' << eta << '\n';
    }
    detector.checkStale(t, tracker);
  }

  log.writeCsv("events.csv");
  std::cout << "Simulation done. " << log.events().size()
            << " events logged -> telemetry.csv, events.csv\n";
  for (const auto& e : log.events())
    std::cout << "  [t=" << e.t << "] " << e.id << " " << e.type << ": " << e.message << '\n';
}
