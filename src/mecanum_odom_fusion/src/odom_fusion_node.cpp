/**
 * Fuses mecanum wheel odometry with rf2o laser odometry using Ekf2D.
 *
 * Both inputs are drifting odometry sources, so their absolute poses are never
 * fused: they would fight each other as they drift apart.
 *
 *  - wheel: body velocity, from the twist (or differentiated pose)
 *  - laser: motion between two consecutive scans, taken from the rf2o POSE.
 *           The ROS1 rf2o twist has linear.y = 0, wrong for a mecanum robot.
 *
 * A laser result describes an interval that is already in the past. The node
 * keeps a short history of the filter, goes back to the previous scan time,
 * replays the wheel data up to the new scan, applies the laser motion there and
 * replays the rest.
 */
#include <algorithm>
#include <deque>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "mecanum_odom_fusion/ekf2d.h"

namespace mecanum_odom_fusion
{

class OdomFusionNode
{
public:
  OdomFusionNode() : pnh_("~")
  {
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param("publish_tf", publish_tf_, true);
    pnh_.param("publish_rate", publish_rate_, 50.0);
    pnh_.param("sensor_timeout", sensor_timeout_, 0.5);

    const Eigen::Vector3d q = vectorParam("process_noise", {0.5, 0.5, 1.0});
    initial_ekf_.setProcessNoise(q(0), q(1), q(2));

    pnh_.param<std::string>("wheel/velocity_source", wheel_source_, "twist");
    if (wheel_source_ != "twist" && wheel_source_ != "pose")
    {
      ROS_WARN("wheel/velocity_source must be 'twist' or 'pose', using 'twist'");
      wheel_source_ = "twist";
    }
    wheel_stddev_ = vectorParam("wheel/stddev", {0.15, 0.20, 0.40});
    pnh_.param("wheel/gate", wheel_gate_, 25.0);
    pnh_.param("wheel/max_consecutive_rejects", wheel_max_rejects_, 5);
    pnh_.param("history_length", history_length_, 2.0);

    pnh_.param("laser/enabled", laser_enabled_, true);
    laser_stddev_ = vectorParam("laser/increment_stddev", {0.005, 0.005, 0.001});
    pnh_.param("laser/gate", laser_gate_, 0.0);
    pnh_.param("laser/max_consecutive_rejects", laser_max_rejects_, 3);
    pnh_.param("laser/max_linear_velocity", max_linear_vel_, 1.5);
    pnh_.param("laser/max_angular_velocity", max_angular_vel_, 3.0);
    pnh_.param("laser/max_interval", laser_max_interval_, 1.0);

    pnh_.param("slip_detection/enabled", slip_enabled_, true);
    pnh_.param("slip_detection/linear_threshold", slip_linear_threshold_, 0.10);
    pnh_.param("slip_detection/angular_threshold", slip_angular_threshold_, 0.20);
    pnh_.param("slip_detection/stddev_scale", slip_stddev_scale_, 5.0);

    wheel_inflation_.setOnes();

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_fused", 10);
    slip_pub_ = pnh_.advertise<std_msgs::Bool>("wheel_slip", 10);
    wheel_sub_ = nh_.subscribe("odom_wheel", 50, &OdomFusionNode::wheelCallback, this,
                               ros::TransportHints().tcpNoDelay());
    if (laser_enabled_)
      laser_sub_ = nh_.subscribe("odom_laser", 10, &OdomFusionNode::laserCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    reset_srv_ = pnh_.advertiseService("reset", &OdomFusionNode::resetCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(publish_rate_, 1.0)),
                             &OdomFusionNode::publishCallback, this);
  }

private:
  /// The filter right after one event, so that it can be replayed from there.
  struct Snapshot
  {
    ros::Time stamp;
    bool has_wheel;               // false: created by a laser measurement
    Eigen::Vector3d wheel;        // wheel velocity applied at this event
    Ekf2D ekf;
    int wheel_rejects;
  };

  struct PoseSample
  {
    bool valid = false;
    ros::Time stamp;
    Eigen::Vector3d pose;
  };

  Eigen::Vector3d vectorParam(const std::string &name, const std::vector<double> &fallback)
  {
    std::vector<double> v;
    pnh_.param(name, v, fallback);
    if (v.size() != 3 || *std::min_element(v.begin(), v.end()) <= 0.0)
    {
      ROS_WARN("~%s must be 3 positive numbers, using the default", name.c_str());
      v = fallback;
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  static PoseSample toPoseSample(const nav_msgs::Odometry &msg)
  {
    PoseSample sample;
    sample.valid = true;
    sample.stamp = msg.header.stamp;
    sample.pose << msg.pose.pose.position.x, msg.pose.pose.position.y,
        tf::getYaw(msg.pose.pose.orientation);
    return sample;
  }

  void resetAll()
  {
    history_.clear();
    last_measurement_time_ = ros::Time();
    wheel_prev_.valid = false;
    laser_prev_.valid = false;
    clone_stamp_ = ros::Time();
    wheel_inflation_.setOnes();
  }

  Snapshot initialSnapshot(const ros::Time &stamp) const
  {
    Snapshot s;
    s.stamp = stamp;
    s.has_wheel = false;
    s.wheel.setZero();
    s.ekf = initial_ekf_;
    s.wheel_rejects = 0;
    return s;
  }

  /// Advance a snapshot in time. An event slightly in the past is applied "now".
  void predictTo(Snapshot &s, const ros::Time &stamp) const
  {
    const double dt = (stamp - s.stamp).toSec();
    if (dt <= 0.0)
      return;
    // Never integrate over a long gap with an old velocity
    if (dt > sensor_timeout_)
      s.ekf.zeroVelocity();
    s.ekf.predict(std::min(dt, sensor_timeout_));
    s.stamp = stamp;
  }

  void applyWheel(Snapshot &s) const
  {
    const Eigen::Vector3d stddev = wheel_stddev_.cwiseProduct(wheel_inflation_);
    const Eigen::Matrix3d R = stddev.array().square().matrix().asDiagonal();

    // After several rejections in a row it is the filter that is wrong: accept
    const bool force = s.wheel_rejects >= wheel_max_rejects_;
    if (s.ekf.updateVelocity(s.wheel, R, force ? 0.0 : wheel_gate_))
    {
      s.wheel_rejects = 0;
    }
    else
    {
      ++s.wheel_rejects;
      ROS_WARN_THROTTLE(1.0, "Rejected wheel outlier: vx %.2f vy %.2f wz %.2f",
                        s.wheel(0), s.wheel(1), s.wheel(2));
    }
  }

  void prune()
  {
    while (history_.size() > 1 &&
           (history_.back().stamp - history_.front().stamp).toSec() > history_length_)
      history_.pop_front();
  }

  bool timeJumpedBack(const ros::Time &stamp)
  {
    if (history_.empty() || (history_.back().stamp - stamp).toSec() <= history_length_ + 1.0)
      return false;
    ROS_WARN("Time jumped backwards, resetting the filter");
    resetAll();
    return true;
  }

  void wheelCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    timeJumpedBack(stamp);
    Eigen::Vector3d velocity;

    if (wheel_source_ == "pose")
    {
      PoseSample current = toPoseSample(*msg);
      current.stamp = stamp;
      const PoseSample previous = wheel_prev_;
      wheel_prev_ = current;
      if (!previous.valid)
        return;
      const double dt = (current.stamp - previous.stamp).toSec();
      if (dt <= 1e-4 || dt > sensor_timeout_)
        return;
      velocity = bodyVelocityFromPoses(previous.pose, current.pose, dt);
    }
    else
    {
      velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
          msg->twist.twist.angular.z;
    }

    if (!velocity.allFinite())
      return;

    // Without recent laser data there is nothing to judge the wheels against
    if (laser_prev_.valid && (stamp - laser_prev_.stamp).toSec() > laser_max_interval_)
      wheel_inflation_.setOnes();

    Snapshot next = history_.empty() ? initialSnapshot(stamp) : history_.back();
    predictTo(next, stamp);
    next.has_wheel = true;
    next.wheel = velocity;
    applyWheel(next);
    history_.push_back(next);
    prune();
    last_measurement_time_ = ros::Time::now();
  }

  void laserCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    const PoseSample current = toPoseSample(*msg);
    if (!current.pose.allFinite())
    {
      ROS_WARN_THROTTLE(1.0, "Laser odometry pose is not finite, ignored");
      laser_prev_.valid = false;
      return;
    }
    timeJumpedBack(current.stamp);

    const PoseSample previous = laser_prev_;
    const double dt = previous.valid ? (current.stamp - previous.stamp).toSec() : 0.0;
    if (previous.valid && dt >= 0.0 && dt < 1e-3)
      return;  // the same scan again
    laser_prev_ = current;

    bool usable = previous.valid && dt > 0.0 && dt <= laser_max_interval_;
    Eigen::Vector3d increment = Eigen::Vector3d::Zero();
    if (usable)
    {
      increment = relativePose(previous.pose, current.pose);
      const double speed = increment.head<2>().norm() / dt;
      const double turn_rate = std::abs(increment(2)) / dt;
      if (speed > max_linear_vel_ || turn_rate > max_angular_vel_)
      {
        ROS_WARN_THROTTLE(1.0, "Implausible laser motion (%.2f m/s, %.2f rad/s), ignored",
                          speed, turn_rate);
        usable = false;
      }
    }

    if (usable && slip_enabled_)
      detectSlip(previous.stamp, current.stamp,
                 bodyVelocityFromPoses(previous.pose, current.pose, dt));

    applyLaser(current.stamp, increment, usable);
  }

  /// Rewind, replay the wheel data and correct at the scan time `stamp`.
  void applyLaser(const ros::Time &stamp, const Eigen::Vector3d &increment, bool usable)
  {
    if (history_.empty())
      history_.push_back(initialSnapshot(stamp));

    // The snapshot of the previous scan holds the cloned pose: restart from it.
    // Otherwise (first scan, gap) restart from the last event before this scan.
    const ros::Time from = usable && !clone_stamp_.isZero() ? clone_stamp_ : stamp;
    int start = static_cast<int>(history_.size()) - 1;
    while (start >= 0 && history_[start].stamp > from)
      --start;
    if (start < 0)
    {
      ROS_WARN_THROTTLE(1.0, "Laser odometry is older than the filter history, ignored");
      clone_stamp_ = ros::Time();
      return;
    }

    std::deque<Snapshot> later(history_.begin() + start + 1, history_.end());
    history_.erase(history_.begin() + start + 1, history_.end());

    auto replay = [this](const Snapshot &event) {
      Snapshot next = history_.back();
      predictTo(next, event.stamp);
      next.has_wheel = event.has_wheel;
      next.wheel = event.wheel;
      if (next.has_wheel)
        applyWheel(next);
      history_.push_back(next);
    };

    while (!later.empty() && later.front().stamp <= stamp)
    {
      replay(later.front());
      later.pop_front();
    }

    Snapshot corrected = history_.back();
    predictTo(corrected, stamp);
    corrected.has_wheel = false;
    if (usable && !clone_stamp_.isZero() && corrected.ekf.hasClone())
    {
      const Eigen::Matrix3d R = laser_stddev_.array().square().matrix().asDiagonal();
      const bool force = laser_rejects_ >= laser_max_rejects_;
      if (corrected.ekf.updateRelativePose(increment, R, force ? 0.0 : laser_gate_))
      {
        laser_rejects_ = 0;
        last_measurement_time_ = ros::Time::now();
      }
      else
      {
        ++laser_rejects_;
        ROS_WARN_THROTTLE(1.0, "Rejected laser outlier");
      }
    }
    corrected.ekf.clonePose();
    clone_stamp_ = corrected.stamp;
    history_.push_back(corrected);

    for (const Snapshot &event : later)
      replay(event);
    prune();
  }

  /**
   * Compare the wheel velocity averaged over the same interval as the laser
   * measurement. An axis that disagrees gets a larger wheel covariance, which
   * is applied while that interval is replayed.
   * This assumes the laser is right when they disagree: turn it off where scan
   * matching is unreliable (long featureless corridors, wide open spaces).
   */
  void detectSlip(const ros::Time &begin, const ros::Time &end, const Eigen::Vector3d &laser)
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int count = 0;
    for (const Snapshot &s : history_)
    {
      if (s.has_wheel && s.stamp > begin && s.stamp <= end)
      {
        sum += s.wheel;
        ++count;
      }
    }

    wheel_inflation_.setOnes();
    bool slip = false;
    if (count > 0)
    {
      const Eigen::Vector3d difference = sum / count - laser;
      const Eigen::Vector3d threshold(slip_linear_threshold_, slip_linear_threshold_,
                                      slip_angular_threshold_);
      for (int i = 0; i < 3; ++i)
      {
        if (std::abs(difference(i)) > threshold(i))
        {
          wheel_inflation_(i) = slip_stddev_scale_;
          slip = true;
        }
      }
    }

    std_msgs::Bool flag;
    flag.data = slip;
    slip_pub_.publish(flag);
  }

  bool resetCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    resetAll();
    ROS_INFO("Filter reset");
    return true;
  }

  void publishCallback(const ros::TimerEvent &)
  {
    if (history_.empty())
      return;

    const ros::Time now = ros::Time::now();
    if (!last_measurement_time_.isZero() &&
        (now - last_measurement_time_).toSec() > sensor_timeout_)
    {
      ROS_WARN_THROTTLE(2.0, "No odometry input for %.1f s, holding the pose", sensor_timeout_);
      history_.back().ekf.zeroVelocity();
    }

    // Extrapolate a copy to "now"; the filter only advances with measurements
    Ekf2D predicted = history_.back().ekf;
    const double dt = (now - history_.back().stamp).toSec();
    predicted.predict(std::max(0.0, std::min(dt, 0.1)));

    // TF rejects repeated stamps (sim time can stand still between two ticks)
    if (now <= last_publish_time_ && (last_publish_time_ - now).toSec() < 1.0)
      return;
    last_publish_time_ = now;

    const Ekf2D::StateVector &x = predicted.state();
    const Ekf2D::StateMatrix &P = predicted.covariance();

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x(Ekf2D::X);
    odom.pose.pose.position.y = x(Ekf2D::Y);
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(x(Ekf2D::YAW));
    odom.twist.twist.linear.x = x(Ekf2D::VX);
    odom.twist.twist.linear.y = x(Ekf2D::VY);
    odom.twist.twist.angular.z = x(Ekf2D::WZ);

    // ROS 6x6 order is [x y z roll pitch yaw]
    const int ros_index[3] = {0, 1, 5};
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        odom.pose.covariance[6 * ros_index[r] + ros_index[c]] = P(r, c);
        odom.twist.covariance[6 * ros_index[r] + ros_index[c]] =
            P(Ekf2D::VX + r, Ekf2D::VX + c);
      }
    }
    for (int i : {2, 3, 4})  // unused axes of a planar robot
    {
      odom.pose.covariance[7 * i] = 1e6;
      odom.twist.covariance[7 * i] = 1e6;
    }
    odom_pub_.publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x(Ekf2D::X);
      transform.transform.translation.y = x(Ekf2D::Y);
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber wheel_sub_, laser_sub_;
  ros::Publisher odom_pub_, slip_pub_;
  ros::ServiceServer reset_srv_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;

  Ekf2D initial_ekf_;
  std::deque<Snapshot> history_;
  ros::Time clone_stamp_, last_measurement_time_, last_publish_time_;
  PoseSample wheel_prev_, laser_prev_;

  std::string odom_frame_, base_frame_, wheel_source_;
  bool publish_tf_, laser_enabled_, slip_enabled_;
  double publish_rate_, sensor_timeout_;
  double wheel_gate_, laser_gate_;
  int wheel_max_rejects_, laser_max_rejects_;
  int laser_rejects_ = 0;
  double history_length_;
  double max_linear_vel_, max_angular_vel_, laser_max_interval_;
  double slip_linear_threshold_, slip_angular_threshold_, slip_stddev_scale_;
  Eigen::Vector3d wheel_stddev_, laser_stddev_, wheel_inflation_;
};

}  // namespace mecanum_odom_fusion

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_fusion");
  mecanum_odom_fusion::OdomFusionNode node;
  ros::spin();
  return 0;
}
