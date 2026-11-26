#ifndef MECANUM_ODOM_FUSION_EKF2D_H
#define MECANUM_ODOM_FUSION_EKF2D_H

#include <Eigen/Dense>

namespace mecanum_odom_fusion
{

/**
 * Planar EKF for a holonomic (mecanum) base.
 *
 * State: [x, y, yaw, vx, vy, wz, cx, cy, cyaw]
 *   x, y, yaw      pose in the odom frame
 *   vx, vy, wz     velocity in the robot body frame
 *   cx, cy, cyaw   copy ("clone") of the pose at the time of the last clonePose()
 *
 * Motion model: constant velocity, driven by white-noise acceleration.
 * Measurements:
 *   - body velocity [vx, vy, wz]                       (wheel odometry)
 *   - motion since the clone [dx, dy, dyaw]            (laser odometry)
 * A relative odometry source only says how far the robot moved between two
 * instants. Keeping the earlier pose in the state ("stochastic cloning") lets
 * the filter apply exactly that, instead of pretending it is a velocity.
 *
 * This class has no ROS dependency so it can be unit tested on its own.
 */
class Ekf2D
{
public:
  static constexpr int SIZE = 9;
  using StateVector = Eigen::Matrix<double, SIZE, 1>;
  using StateMatrix = Eigen::Matrix<double, SIZE, SIZE>;
  using MeasurementJacobian = Eigen::Matrix<double, 3, SIZE>;

  enum Index { X = 0, Y, YAW, VX, VY, WZ, CX, CY, CYAW };

  Ekf2D();

  /// Back to the origin, standing still.
  void reset();

  /// Spectral density of the acceleration noise: x, y [m^2/s^3], yaw [rad^2/s^3].
  /// Larger values trust the measurements more and the motion model less.
  void setProcessNoise(double accel_x, double accel_y, double accel_yaw);

  void predict(double dt);

  /**
   * Correct with a body velocity measurement.
   * @param z     [vx, vy, wz]
   * @param R     measurement covariance
   * @param gate  reject if the squared Mahalanobis distance exceeds this (<= 0: off)
   * @return false if the measurement was rejected; the filter is then unchanged
   */
  bool updateVelocity(const Eigen::Vector3d &z, const Eigen::Matrix3d &R, double gate = 0.0);

  /// Remember the current pose as the reference of the next relative update.
  void clonePose();
  void dropClone() { has_clone_ = false; }
  bool hasClone() const { return has_clone_; }

  /**
   * Correct with the motion since clonePose().
   * @param z     [dx, dy, dyaw], dx and dy in the body frame of the cloned pose
   * @param R     measurement covariance
   * @param gate  see updateVelocity()
   * @return false if there is no clone or the measurement was rejected
   */
  bool updateRelativePose(const Eigen::Vector3d &z, const Eigen::Matrix3d &R, double gate = 0.0);

  /// Used when every sensor has gone silent: stop integrating the last velocity.
  void zeroVelocity();

  const StateVector &state() const { return x_; }
  const StateMatrix &covariance() const { return P_; }

  static double wrapAngle(double angle);

private:
  bool correct(const MeasurementJacobian &H, const Eigen::Vector3d &innovation,
               const Eigen::Matrix3d &R, double gate);

  bool has_clone_;
  StateVector x_;
  StateMatrix P_;
  Eigen::Vector3d q_;
};

/// Motion from pose0 to pose1 as [dx, dy, dyaw], dx and dy in the frame of pose0.
Eigen::Vector3d relativePose(const Eigen::Vector3d &pose0, const Eigen::Vector3d &pose1);

/**
 * Average body velocity that moves pose0 to pose1 in dt seconds.
 * Poses are [x, y, yaw] in any common frame.
 */
Eigen::Vector3d bodyVelocityFromPoses(const Eigen::Vector3d &pose0,
                                      const Eigen::Vector3d &pose1, double dt);

}  // namespace mecanum_odom_fusion

#endif
