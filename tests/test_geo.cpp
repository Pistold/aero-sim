#include "aero/geo.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace aero;

TEST(Geo, ZeroDistance) { EXPECT_NEAR(distanceM(10, 20, 10, 20), 0.0, 1e-6); }

TEST(Geo, OneDegreeLatitude) {
  EXPECT_NEAR(distanceM(0, 0, 1, 0), 111194.9, 50.0);
}

TEST(Geo, KnownCityPair) {  // Boston -> New York, ~306 km
  EXPECT_NEAR(distanceM(42.3601, -71.0589, 40.7128, -74.0060) / 1000.0, 306.0, 3.0);
}

TEST(Geo, CardinalBearings) {
  EXPECT_NEAR(bearingDeg(0, 0, 1, 0), 0.0, 1e-6);
  EXPECT_NEAR(bearingDeg(0, 0, 0, 1), 90.0, 1e-6);
  EXPECT_NEAR(bearingDeg(0, 0, -1, 0), 180.0, 1e-6);
  EXPECT_NEAR(bearingDeg(0, 0, 0, -1), 270.0, 1e-6);
}

TEST(Geo, DestinationRoundTrip) {
  double lat, lon;
  destinationPoint(42.0, -71.0, 45.0, 10000.0, lat, lon);
  EXPECT_NEAR(distanceM(42.0, -71.0, lat, lon), 10000.0, 0.5);
  EXPECT_NEAR(bearingDeg(42.0, -71.0, lat, lon), 45.0, 0.05);
}

TEST(Geo, Eta) {
  EXPECT_DOUBLE_EQ(etaSeconds(1000, 100), 10.0);
  EXPECT_TRUE(std::isinf(etaSeconds(1000, 0)));
}
