#include "mecanum_odom_fusion/ekf2d.h"

#include <cmath>

namespace mecanum_odom_fusion
{

Ekf2D::Ekf2D() : q_(1.0, 1.0, 2.0)
{
  reset();
}

void Ekf2D::reset()
{
  x_.setZero();
  P_.setZero();
  P_.diagonal() << 1e-6, 1e-6, 1e-6, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0;
  has_clone_ = false;
}

void Ekf2D::setProcessNoise(double accel_x, double accel_y, double accel_yaw)
{
  q_ << accel_x, accel_y, accel_yaw;
}

void Ekf2D::predict(double dt)
{
  if (dt <= 0.0)
    return;

  const double vx = x_(VX);
  const double vy = x_(VY);
  const double wz = x_(WZ);

  // Heading at the middle of the step (second order accurate on arcs)
  const double heading = x_(YAW) + 0.5 * wz * dt;
  const double c = std::cos(heading);
  const double s = std::sin(heading);

  const double dx = (vx * c - vy * s) * dt;
  const double dy = (vx * s + vy * c) * dt;

  x_(X) += dx;
  x_(Y) += dy;
  x_(YAW) = wrapAngle(x_(YAW) + wz * dt);

  // Jacobian of the motion model
  StateMatrix F = StateMatrix::Identity();
  F(X, YAW) = -dy;
  F(X, VX) = c * dt;
  F(X, VY) = -s * dt;
  F(X, WZ) = -0.5 * dy * dt;
  F(Y, YAW) = dx;
  F(Y, VX) = s * dt;
  F(Y, VY) = c * dt;
  F(Y, WZ) = 0.5 * dx * dt;
  F(YAW, WZ) = dt;

  // Process noise: white-noise acceleration in the body frame, rotated to odom
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.topLeftCorner<2, 2>() << c, -s, s, c;
  const Eigen::Matrix3d Qa = q_.asDiagonal();

  StateMatrix Q = StateMatrix::Zero();
  Q.block<3, 3>(X, X) = R * Qa * R.transpose() * (dt * dt * dt / 3.0);
  const Eigen::Matrix3d Q_pose_velocity = R * Qa * (dt * dt / 2.0);
  Q.block<3, 3>(X, VX) = Q_pose_velocity;
  Q.block<3, 3>(VX, X) = Q_pose_velocity.transpose();
  Q.block<3, 3>(VX, VX) = Qa * dt;

  P_ = F * P_ * F.transpose() + Q;
}

bool Ekf2D::updateVelocity(const Eigen::Vector3d &z, const Eigen::Matrix3d &R, double gate)
{
  MeasurementJacobian H = MeasurementJacobian::Zero();
  H.block<3, 3>(0, VX).setIdentity();
  return correct(H, z - H * x_, R, gate);
}

void Ekf2D::clonePose()
{
  x_.segment<3>(CX) = x_.segment<3>(X);

  // P = M P M^T where M copies the pose rows into the clone rows
  StateMatrix M = StateMatrix::Identity();
  M.block<3, 3>(CX, CX).setZero();
  M.block<3, 3>(CX, X).setIdentity();
  P_ = (M * P_ * M.transpose()).eval();
  has_clone_ = true;
}

bool Ekf2D::updateRelativePose(const Eigen::Vector3d &z, const Eigen::Matrix3d &R, double gate)
{
  if (!has_clone_)
    return false;

  const Eigen::Vector3d predicted = relativePose(x_.segment<3>(CX), x_.segment<3>(X));
  const double c = std::cos(x_(CYAW));
  const double s = std::sin(x_(CYAW));

  MeasurementJacobian H = MeasurementJacobian::Zero();
  H(0, X) = c;
  H(0, Y) = s;
  H(0, CX) = -c;
  H(0, CY) = -s;
  H(0, CYAW) = predicted(1);
  H(1, X) = -s;
  H(1, Y) = c;
  H(1, CX) = s;
  H(1, CY) = -c;
  H(1, CYAW) = -predicted(0);
  H(2, YAW) = 1.0;
  H(2, CYAW) = -1.0;

  Eigen::Vector3d innovation = z - predicted;
  innovation(2) = wrapAngle(innovation(2));
  return correct(H, innovation, R, gate);
}

bool Ekf2D::correct(const MeasurementJacobian &H, const Eigen::Vector3d &innovation,
                    const Eigen::Matrix3d &R, double gate)
{
  if (!innovation.allFinite() || !R.allFinite())
    return false;

  const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
  const Eigen::LDLT<Eigen::Matrix3d> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive())
    return false;

  if (gate > 0.0 && innovation.dot(S_ldlt.solve(innovation)) > gate)
    return false;

  // K = P H^T S^-1  (S and P are symmetric)
  const Eigen::Matrix<double, SIZE, 3> K = S_ldlt.solve(H * P_).transpose();

  x_ += K * innovation;
  x_(YAW) = wrapAngle(x_(YAW));
  x_(CYAW) = wrapAngle(x_(CYAW));

  // Joseph form keeps P symmetric positive semi-definite
  const StateMatrix I_KH = StateMatrix::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose()).eval();
  return true;
}

void Ekf2D::zeroVelocity()
{
  x_.segment<3>(VX).setZero();
  // The velocity is now unrelated to the pose errors accumulated so far
  for (int other : {static_cast<int>(X), static_cast<int>(CX)})
  {
    P_.block<3, 3>(other, VX).setZero();
    P_.block<3, 3>(VX, other).setZero();
  }
}

double Ekf2D::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d relativePose(const Eigen::Vector3d &pose0, const Eigen::Vector3d &pose1)
{
  const double c = std::cos(pose0(2));
  const double s = std::sin(pose0(2));
  const double dx = pose1(0) - pose0(0);
  const double dy = pose1(1) - pose0(1);
  return Eigen::Vector3d(c * dx + s * dy, -s * dx + c * dy, Ekf2D::wrapAngle(pose1(2) - pose0(2)));
}

Eigen::Vector3d bodyVelocityFromPoses(const Eigen::Vector3d &pose0,
                                      const Eigen::Vector3d &pose1, double dt)
{
  const double dyaw = Ekf2D::wrapAngle(pose1(2) - pose0(2));
  const double heading = pose0(2) + 0.5 * dyaw;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double dx = pose1(0) - pose0(0);
  const double dy = pose1(1) - pose0(1);

  return Eigen::Vector3d((c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt);
}

}  // namespace mecanum_odom_fusion
