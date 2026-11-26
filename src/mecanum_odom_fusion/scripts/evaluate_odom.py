#!/usr/bin/env python3
"""
Compare odometry topics with the simulator ground truth.

Start it before driving, stop it with Ctrl-C (or set ~duration, sim seconds):
  rosrun mecanum_odom_fusion evaluate_odom.py
All trajectories are aligned at their first sample, then compared.
"""
import math
import signal
from bisect import bisect_left

import rospy
from nav_msgs.msg import Odometry


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_of(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return msg.pose.pose.position.x, msg.pose.pose.position.y, yaw


def relative(origin, pose):
    """pose expressed in the frame of origin."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2])


def main():
    # shells start background jobs with SIGINT ignored: restore it so Ctrl-C/kill -INT works
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rospy.init_node("evaluate_odom", anonymous=True, disable_signals=True)
    truth_topic = rospy.get_param("~truth", "/ground_truth/odom")
    topics = rospy.get_param("~topics", ["/odom", "/odom_rf2o", "/odom_fused"])
    duration = rospy.get_param("~duration", 0.0)

    data = {t: [] for t in [truth_topic] + topics}

    def store(msg, topic):
        data[topic].append((msg.header.stamp.to_sec(), pose_of(msg)))

    subs = [rospy.Subscriber(t, Odometry, store, t, queue_size=200) for t in data]

    try:
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            if duration > 0 and (rospy.Time.now() - start).to_sec() > duration:
                break
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    for s in subs:
        s.unregister()

    truth = list(data[truth_topic])
    if len(truth) < 2:
        print("No ground truth received on " + truth_topic)
        return
    stamps = [t for t, _ in truth]

    def truth_at(t):
        i = bisect_left(stamps, t)
        if i == 0 or i == len(stamps):
            return None
        (t0, a), (t1, b) = truth[i - 1], truth[i]
        r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]),
                wrap(a[2] + r * wrap(b[2] - a[2])))

    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for (_, a), (_, b) in zip(truth, truth[1:]))
    print("\nground truth path length: %.2f m" % length)
    print("%-14s %12s %12s %12s %12s %12s" % (
        "topic", "pos RMSE m", "pos final m", "yaw RMSE deg", "yaw fin deg", "samples"))

    for topic in topics:
        samples = [(t, p, truth_at(t)) for t, p in list(data[topic])]
        samples = [s for s in samples if s[2] is not None]
        if not samples:
            print("%-14s no data" % topic)
            continue
        est0, ref0 = samples[0][1], samples[0][2]
        pos, ang = [], []
        for _, p, g in samples:
            e, r = relative(est0, p), relative(ref0, g)
            pos.append(math.hypot(e[0] - r[0], e[1] - r[1]))
            ang.append(abs(math.degrees(wrap(e[2] - r[2]))))
        rmse = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
        print("%-14s %12.3f %12.3f %12.2f %12.2f %12d" % (
            topic, rmse(pos), pos[-1], rmse(ang), ang[-1], len(samples)))


if __name__ == "__main__":
    main()
