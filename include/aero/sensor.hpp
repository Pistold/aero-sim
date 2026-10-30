#pragma once
#include "aero/aircraft.hpp"
#include <optional>
#include <random>
#include <string>

namespace aero {

struct SensorReading {
  double t = 0;
  std::string id;
  double lat = 0, lon = 0, altM = 0, speedMps = 0;
};

// Simulated radar/ADS-B style sensor: Gaussian noise + random dropouts.
class Sensor {
 public:
  Sensor(unsigned seed, double posSigmaM = 20.0, double altSigmaM = 10.0,
         double speedSigmaMps = 2.0, double dropoutProb = 0.01);

  // Returns nullopt on a dropout.
  std::optional<SensorReading> observe(double t, const AircraftState& truth);

 private:
  std::mt19937 rng_;
  double posSigmaM_, altSigmaM_, speedSigmaMps_, dropoutProb_;
};

}  // namespace aero
