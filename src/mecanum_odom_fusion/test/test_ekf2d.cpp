#include <cmath>
#include <gtest/gtest.h>

#include "mecanum_odom_fusion/ekf2d.h"

using mecanum_odom_fusion::Ekf2D;
using mecanum_odom_fusion::bodyVelocityFromPoses;

namespace
{
const double kDt = 0.02;

Eigen::Matrix3d diag(double stddev)
{
  return Eigen::Matrix3d::Identity() * stddev * stddev;
}

void drive(Ekf2D &ekf, const Eigen::Vector3d &velocity, double seconds)
{
  for (int i = 0; i < std::lround(seconds / kDt); ++i)
  {
    ekf.predict(kDt);
    ASSERT_TRUE(ekf.updateVelocity(velocity, diag(1e-4)));
  }
}
}  // namespace

TEST(Ekf2D, strafingMovesAlongY)
{
  Ekf2D ekf;
  drive(ekf, Eigen::Vector3d(0.0, 0.5, 0.0), 4.0);
  EXPECT_NEAR(ekf.state()(Ekf2D::X), 0.0, 1e-3);
  EXPECT_NEAR(ekf.state()(Ekf2D::Y), 2.0, 0.02);
}

TEST(Ekf2D, holonomicArcMatchesClosedForm)
{
  Ekf2D ekf;
  const double vx = 0.3, vy = 0.2, wz = 0.5, t = 6.0;
  drive(ekf, Eigen::Vector3d(vx, vy, wz), t);
  const double x = (vx * std::sin(wz * t) + vy * (std::cos(wz * t) - 1.0)) / wz;
  const double y = (vx * (1.0 - std::cos(wz * t)) + vy * std::sin(wz * t)) / wz;
  EXPECT_NEAR(ekf.state()(Ekf2D::X), x, 0.02);
  EXPECT_NEAR(ekf.state()(Ekf2D::Y), y, 0.02);
  EXPECT_NEAR(Ekf2D::wrapAngle(ekf.state()(Ekf2D::YAW) - wz * t), 0.0, 0.02);
}

TEST(Ekf2D, moreAccurateSourceDominates)
{
  Ekf2D ekf;
  for (int i = 0; i < 200; ++i)
  {
    ekf.predict(kDt);
    ekf.updateVelocity(Eigen::Vector3d(1.0, 0.0, 0.0), diag(0.3));
    ekf.updateVelocity(Eigen::Vector3d(0.5, 0.0, 0.0), diag(0.03));
  }
  EXPECT_NEAR(ekf.state()(Ekf2D::VX), 0.5, 0.02);
}

TEST(Ekf2D, gateRejectsOutlierAndKeepsState)
{
  Ekf2D ekf;
  drive(ekf, Eigen::Vector3d(0.2, 0.0, 0.0), 1.0);
  const Ekf2D::StateVector before = ekf.state();
  EXPECT_FALSE(ekf.updateVelocity(Eigen::Vector3d(5.0, 0.0, 0.0), diag(0.05), 16.0));
  EXPECT_TRUE(ekf.state().isApprox(before));
}

TEST(Ekf2D, rejectsNan)
{
  Ekf2D ekf;
  EXPECT_FALSE(ekf.updateVelocity(Eigen::Vector3d(NAN, 0.0, 0.0), diag(0.05)));
  EXPECT_TRUE(ekf.state().allFinite());
}

TEST(Ekf2D, covarianceStaysSymmetricPositive)
{
  Ekf2D ekf;
  drive(ekf, Eigen::Vector3d(0.3, -0.2, 0.4), 20.0);
  const Ekf2D::StateMatrix P = ekf.covariance();
  EXPECT_TRUE(P.isApprox(P.transpose()));
  Eigen::SelfAdjointEigenSolver<Ekf2D::StateMatrix> solver(P);
  EXPECT_GT(solver.eigenvalues().minCoeff(), -1e-12);
}

TEST(Ekf2D, zeroVelocityHoldsPose)
{
  Ekf2D ekf;
  drive(ekf, Eigen::Vector3d(0.3, 0.1, 0.2), 1.0);
  ekf.zeroVelocity();
  const Ekf2D::StateVector before = ekf.state();
  ekf.predict(1.0);
  EXPECT_TRUE(ekf.state().isApprox(before));
}

TEST(Ekf2D, relativePoseCorrectsBiasedWheelsOverTheInterval)
{
  // Truth: straight line at 0.5 m/s. Wheels report a false yaw rate.
  Ekf2D wheels_only, fused;
  const Eigen::Matrix3d wheel_R = Eigen::Vector3d(0.15, 0.2, 0.4).array().square().matrix().asDiagonal();
  const Eigen::Matrix3d laser_R = Eigen::Vector3d(0.005, 0.005, 0.003).array().square().matrix().asDiagonal();
  fused.clonePose();
  for (int i = 1; i <= 500; ++i)
  {
    for (Ekf2D *ekf : {&wheels_only, &fused})
    {
      ekf->predict(kDt);
      ekf->updateVelocity(Eigen::Vector3d(0.5, 0.0, 0.15), wheel_R);
    }
    if (i % 10 == 0)  // laser at 5 Hz
    {
      ASSERT_TRUE(fused.updateRelativePose(Eigen::Vector3d(0.1, 0.0, 0.0), laser_R));
      fused.clonePose();
    }
  }
  EXPECT_GT(std::abs(wheels_only.state()(Ekf2D::YAW)), 1.0);
  EXPECT_NEAR(fused.state()(Ekf2D::YAW), 0.0, 0.05);
  EXPECT_NEAR(fused.state()(Ekf2D::X), 5.0, 0.1);
  EXPECT_NEAR(fused.state()(Ekf2D::Y), 0.0, 0.15);
}

TEST(Ekf2D, relativeUpdateNeedsAClone)
{
  Ekf2D ekf;
  EXPECT_FALSE(ekf.updateRelativePose(Eigen::Vector3d::Zero(), diag(0.01)));
  ekf.clonePose();
  ekf.predict(kDt);
  EXPECT_TRUE(ekf.updateRelativePose(Eigen::Vector3d::Zero(), diag(0.01)));
}

TEST(RelativePose, isExpressedInTheStartFrame)
{
  // Facing +y, moving towards -x is a move to the left
  const Eigen::Vector3d d = mecanum_odom_fusion::relativePose(
      Eigen::Vector3d(1.0, 1.0, M_PI / 2), Eigen::Vector3d(0.0, 1.0, M_PI / 2));
  EXPECT_TRUE(d.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-9));
}

TEST(BodyVelocity, recoversVelocityAcrossAngleWrap)
{
  // Robot heading close to +pi, strafing left while rotating
  const double vx = 0.1, vy = 0.4, wz = 0.5, dt = 0.05, yaw0 = 3.13;
  const double mid = yaw0 + 0.5 * wz * dt;
  const Eigen::Vector3d p0(1.0, 2.0, yaw0);
  const Eigen::Vector3d p1(1.0 + (vx * std::cos(mid) - vy * std::sin(mid)) * dt,
                           2.0 + (vx * std::sin(mid) + vy * std::cos(mid)) * dt,
                           Ekf2D::wrapAngle(yaw0 + wz * dt));
  EXPECT_TRUE(bodyVelocityFromPoses(p0, p1, dt).isApprox(Eigen::Vector3d(vx, vy, wz), 1e-9));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
