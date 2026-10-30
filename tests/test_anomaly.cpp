#include "aero/anomaly.hpp"
#include <gtest/gtest.h>

using namespace aero;

static SensorReading rd(double t, double alt, double spd) {
  SensorReading r; r.t = t; r.id = "X"; r.altM = alt; r.speedMps = spd; return r;
}

TEST(Anomaly, NormalDataRaisesNothing) {
  EventLog log; AnomalyDetector d(log);
  d.checkReading(rd(0, 5000, 200));
  d.checkReading(rd(1, 5005, 201));
  EXPECT_TRUE(log.events().empty());
}

TEST(Anomaly, AltitudeJumpDetected) {
  EventLog log; AnomalyDetector d(log);
  d.checkReading(rd(0, 5000, 200));
  d.checkReading(rd(1, 7500, 200));
  ASSERT_EQ(log.events().size(), 1u);
  EXPECT_EQ(log.events()[0].type, "ALT_JUMP");
}

TEST(Anomaly, SpeedViolationLoggedOnceUntilCleared) {
  EventLog log; AnomalyDetector d(log);
  d.checkReading(rd(0, 5000, 400));
  d.checkReading(rd(1, 5000, 410));
  EXPECT_EQ(log.events().size(), 1u);
  d.checkReading(rd(2, 5000, 200));  // back to normal
  d.checkReading(rd(3, 5000, 400));  // new violation
  EXPECT_EQ(log.events().size(), 2u);
}

TEST(Anomaly, StaleTrackFlagsDropout) {
  EventLog log; AnomalyDetector d(log); Tracker tr;
  auto r = rd(0, 5000, 200);
  tr.update(r);
  d.checkStale(3, tr);
  EXPECT_TRUE(log.events().empty());
  d.checkStale(10, tr);
  ASSERT_EQ(log.events().size(), 1u);
  EXPECT_EQ(log.events()[0].type, "DATA_DROPOUT");
}
