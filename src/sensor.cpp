#include "aero/sensor.hpp"
#include "aero/geo.hpp"
#include <cmath>

namespace aero {

Sensor::Sensor(unsigned seed, double p, double a, double s, double d)
    : rng_(seed), posSigmaM_(p), altSigmaM_(a), speedSigmaMps_(s), dropoutProb_(d) {}

std::optional<SensorReading> Sensor::observe(double t, const AircraftState& truth) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  if (u(rng_) < dropoutProb_) return std::nullopt;

  std::normal_distribution<double> n(0.0, 1.0);
  const double mPerDegLat = kPi * kEarthRadiusM / 180.0;
  const double mPerDegLon = mPerDegLat * std::cos(deg2rad(truth.lat));

  SensorReading r;
  r.t = t;
  r.id = truth.id;
  r.lat = truth.lat + n(rng_) * posSigmaM_ / mPerDegLat;
  r.lon = truth.lon + n(rng_) * posSigmaM_ / mPerDegLon;
  r.altM = truth.altM + n(rng_) * altSigmaM_;
  r.speedMps = truth.speedMps + n(rng_) * speedSigmaMps_;
  return r;
}

}  // namespace aero
