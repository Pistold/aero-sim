#include "aero/geo.hpp"
#include <cmath>

namespace aero {

double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }

double distanceM(double lat1, double lon1, double lat2, double lon2) {
  const double p1 = deg2rad(lat1), p2 = deg2rad(lat2);
  const double dp = p2 - p1, dl = deg2rad(lon2 - lon1);
  const double a = std::sin(dp / 2) * std::sin(dp / 2) +
                   std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
  return 2.0 * kEarthRadiusM * std::asin(std::sqrt(a));
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  const double p1 = deg2rad(lat1), p2 = deg2rad(lat2);
  const double dl = deg2rad(lon2 - lon1);
  const double y = std::sin(dl) * std::cos(p2);
  const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
  double b = rad2deg(std::atan2(y, x));
  return std::fmod(b + 360.0, 360.0);
}

void destinationPoint(double lat, double lon, double bearing, double distM,
                      double& outLat, double& outLon) {
  const double d = distM / kEarthRadiusM, br = deg2rad(bearing);
  const double p1 = deg2rad(lat), l1 = deg2rad(lon);
  const double p2 = std::asin(std::sin(p1) * std::cos(d) +
                              std::cos(p1) * std::sin(d) * std::cos(br));
  const double l2 = l1 + std::atan2(std::sin(br) * std::sin(d) * std::cos(p1),
                                    std::cos(d) - std::sin(p1) * std::sin(p2));
  outLat = rad2deg(p2);
  outLon = std::fmod(rad2deg(l2) + 540.0, 360.0) - 180.0;
}

double etaSeconds(double distM, double speedMps) {
  if (speedMps <= 0.0) return std::numeric_limits<double>::infinity();
  return distM / speedMps;
}

}  // namespace aero
