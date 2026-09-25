#include <gtest/gtest.h>

TEST(ChauffeurMath, ConstantVelocityHasZeroLongitudinalJerk)
{
  const double dt = 0.1;
  const double v0 = 4.0;
  const double v1 = 4.0;
  const double v2 = 4.0;
  const double a0 = (v1 - v0) / dt;
  const double a1 = (v2 - v1) / dt;
  const double jerk = (a1 - a0) / dt;
  EXPECT_DOUBLE_EQ(jerk, 0.0);
}

TEST(ChauffeurMath, LateralAccelerationScalesWithVelocitySquaredAtFixedCurvature)
{
  const double curvature = 0.1;
  const double slow_v = 3.0;
  const double fast_v = 6.0;
  const double slow_ay = slow_v * (slow_v * curvature);
  const double fast_ay = fast_v * (fast_v * curvature);
  EXPECT_DOUBLE_EQ(fast_ay / slow_ay, 4.0);
}

TEST(ChauffeurMath, NormalizedSteeringIsOneAtMinimumTurningRadius)
{
  const double min_turning_radius = 5.0;
  const double v = 4.0;
  const double wz = v / min_turning_radius;
  const double normalized = min_turning_radius * wz / v;
  EXPECT_NEAR(normalized, 1.0, 1e-12);
}
