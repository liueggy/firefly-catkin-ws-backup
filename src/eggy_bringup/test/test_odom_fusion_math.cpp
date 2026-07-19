#include <gtest/gtest.h>

#include <cmath>

#include "eggy_bringup/odom_fusion_math.h"

TEST(OdomFusionMathTest, CorrectsAcrossAngleWrapUsingShortestPath) {
  const double corrected = eggy_bringup::CorrectYawTowardWheel(
      179.0 * M_PI / 180.0, -179.0 * M_PI / 180.0, 1.0, 0.5);
  EXPECT_NEAR(std::abs(corrected), M_PI, 1e-6);
}

TEST(OdomFusionMathTest, BoundsCorrectionGainForLongTimestampGap) {
  const double corrected =
      eggy_bringup::CorrectYawTowardWheel(0.0, 1.0, 10.0, 1.0);
  EXPECT_NEAR(corrected, 1.0, 1e-9);
}

TEST(OdomFusionMathTest, LearnsBiasGraduallyOnlyWhenCalled) {
  EXPECT_NEAR(eggy_bringup::LearnStationaryGyroBias(0.0, 0.1, 0.02),
              0.002, 1e-9);
}

TEST(OdomFusionMathTest, ProjectsOmniVelocityUsingFusedHeading) {
  const auto world = eggy_bringup::BodyVelocityToWorld(1.0, 0.5, M_PI_2);
  EXPECT_NEAR(world.first, -0.5, 1e-9);
  EXPECT_NEAR(world.second, 1.0, 1e-9);
}
