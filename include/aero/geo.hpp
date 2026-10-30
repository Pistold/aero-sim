#pragma once
#include <limits>

namespace aero {

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg);
double rad2deg(double rad);

// Great-circle (haversine) distance in meters.
double distanceM(double lat1, double lon1, double lat2, double lon2);

// Initial bearing from point 1 to point 2, degrees in [0, 360).
double bearingDeg(double lat1, double lon1, double lat2, double lon2);

// Point reached by travelling distM along bearing from (lat, lon).
void destinationPoint(double lat, double lon, double bearing, double distM,
                      double& outLat, double& outLon);

// Estimated time of arrival in seconds; infinity if speed <= 0.
double etaSeconds(double distM, double speedMps);

}  // namespace aero
