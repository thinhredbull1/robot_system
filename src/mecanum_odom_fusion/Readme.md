# mecanum_odom_fusion

Custom C++ EKF (Eigen, no `robot_localization`) that fuses mecanum wheel odometry
with `rf2o_laser_odometry`. ROS Noetic.

```
/odom      (wheel, ~50 Hz) ───────────────┐
                                          ├─> odom_fusion ─> /odom_fused (50 Hz) + TF odom -> base
/scan ─> rf2o ─> /odom_rf2o (lidar rate) ─┘
```

## rf2o setup

Use the `ros1` branch and apply the patch in this package:

```bash
cd src/rf2o_laser_odometry
git checkout ros1
git apply ../mecanum_odom_fusion/patches/rf2o_ros1.patch
```

The patch:
- stamps `/odom_rf2o` with the **scan time** instead of `ros::Time::now()`;
- reads the laser TF when the first scan arrives. Unpatched, it is looked up
  before the laser frame name is known, so the lidar mounting pose is ignored;
- throttles the "Waiting for laser_scans" warning.

## Build and test

```bash
catkin build rf2o_laser_odometry mecanum_odom_fusion
catkin build mecanum_odom_fusion --no-deps --make-args run_tests
catkin_test_results build/mecanum_odom_fusion
```

## Run

Simulation (`base_footprint`):

```bash
roslaunch nexus_4wd_mecanum_gazebo gazebo_sim.launch            # odom_noise:=false for clean wheel odom
roslaunch mecanum_odom_fusion sim_odom_fusion.launch
rosrun mecanum_odom_fusion evaluate_odom.py                     # Ctrl-C to print the result
rosrun mecanum_odom_fusion drive_test.py _loops:=2              # moves the robot via /cmd_vel
```

Real robot (`base_link`): `roslaunch mecanum_odom_fusion odom_fusion.launch`

Only one node may publish `odom -> base`. Keep `publish_tf` off in the wheel
driver and in rf2o, or start this node with `publish_tf:=false`.

Reset the pose: `rosservice call /odom_fusion/reset`

## How the filter works

State: `[x, y, yaw, vx, vy, wz]` plus a copy of the pose at the previous scan.
Motion model: constant body velocity with white-noise acceleration.

| Source | Used as |
|---|---|
| wheel | body velocity `vx, vy, wz` |
| rf2o  | motion **between two scans** `dx, dy, dyaw`, computed from its pose |

- Absolute poses are never fused: both sources drift.
- The rf2o twist is not used, because it reports `linear.y = 0`.
- A lidar result describes a past interval. The node returns to the previous scan
  in a short history, replays the wheel data, applies the lidar motion at the scan
  time, then replays the rest.
- **Slip detection:** if the mean wheel velocity between two scans differs from the
  lidar by more than a threshold, that axis of the wheel data gets a larger
  covariance over that interval. `/odom_fusion/wheel_slip` shows the verdict.
- With no input for `sensor_timeout`, the pose is held.

## Tuning (`config/ekf.yaml`)

A smaller standard deviation means more trust.

| Symptom | Change |
|---|---|
| Follows wheel drift or slip | raise `wheel/stddev` or lower `laser/increment_stddev` |
| Follows scan matching errors | raise `laser/increment_stddev` |
| Output lags in fast manoeuvres | raise `process_noise` |
| Wheels flagged as slipping all the time | raise the `slip_detection` thresholds |

The values were tuned **in simulation only**. Tune them again on the real robot
(record `/odom` and `/odom_rf2o` with rosbag and replay).

## Simulation results (lidar 5 Hz, wheel noise on)

One fresh run with the final settings, 14.2 m driven, 13 simulated slips:

| Topic | Position RMSE | Yaw RMSE | Final yaw error |
|---|---|---|---|
| `/odom` (wheel) | 0.241 m | 31.6° | 47.6° |
| `/odom_rf2o`    | 0.049 m | 1.2°  | 1.4°  |
| `/odom_fused`   | 0.054 m | 1.2°  | 1.4°  |

In this simulation the fused result is about as accurate as rf2o (slightly better
or slightly worse depending on the run). What fusion adds is 50 Hz output with a
lateral velocity, and operation when one source stops:

- lidar only (wheel topic removed): position RMSE 0.050 m;
- lidar stops halfway: continues on wheels, and drifts like them from then on
  (0.140 m RMSE over the whole run, 31° final yaw error).

With clean wheel odometry (`odom_noise:=false`) the wheels are **better** than the
fused output (0.008 m against 0.025 m), because the tuning assumes noisy wheels.

## Limitations

- With two sources the filter cannot tell which one is wrong. Slip detection
  assumes the lidar is right. In a long featureless corridor or a large open area,
  set `slip_detection/enabled: false` and raise `laser/increment_stddev`.
- There is no absolute reference, so the pose still drifts slowly. Use AMCL or
  SLAM for `map -> odom`.
- No IMU input yet. A gyro would be the most useful addition for heading.

## Real robot: check the yaw sign first

`mecanum_driver/src/mecanum_node.cpp` integrates the pose with `+dtheta` but
publishes `twist.angular.z = -speed_filter_z_`. One of the two has the wrong sign.
Rotate the robot counter-clockwise (yaw must increase, `angular.z` must be
positive) and fix the driver. If only the twist is wrong, you can also set
`wheel/velocity_source: pose`.

## Simulated wheel noise

The Gazebo plugin `nexus_ros_force_based_move` changes only the published `/odom`,
not the robot motion: scale errors, speed-dependent gaussian noise, false yaw
while translating, and random slip bursts. The parameters are the `odom*` tags in
`nexus_4wd_mecanum.gazebo`. Set `odomNoiseSeed` to a non-zero value for a
repeatable random sequence. The true pose is published on `/ground_truth/odom`.
