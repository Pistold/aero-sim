#include "aero/aircraft.hpp"
#include "aero/geo.hpp"
#include <gtest/gtest.h>

using namespace aero;

TEST(Aircraft, MovesTowardDestinationAndArrives) {
  Aircraft ac({"T1", 0, 0, 1000, 100, 0}, {0, 0.1}, 1000);
  const double d0 = distanceM(0, 0, 0, 0.1);
  ac.step(10);
  EXPECT_LT(distanceM(ac.state().lat, ac.state().lon, 0, 0.1), d0);
  for (int i = 0; i < 1000 && !ac.arrived(); ++i) ac.step(10);
  EXPECT_TRUE(ac.arrived());
}

TEST(Aircraft, ClimbsAtLimitedRate) {
  Aircraft ac({"T2", 0, 0, 1000, 100, 0}, {1, 1}, 2000, 10.0);
  ac.step(5);
  EXPECT_NEAR(ac.state().altM, 1050.0, 1e-9);
}
