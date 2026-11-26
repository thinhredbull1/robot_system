#!/usr/bin/env python3
"""Drive a repeatable mecanum pattern on /cmd_vel (timed with ROS/sim time)."""
import rospy
from geometry_msgs.msg import Twist

# (duration s, vx, vy, wz). The pattern ends close to where it started.
PATTERN = [
    (2.0, 0.0, 0.0, 0.0),
    (4.0, 0.25, 0.0, 0.0),    # forward
    (3.0, 0.0, 0.25, 0.0),    # strafe left
    (4.0, -0.25, 0.0, 0.0),   # backward
    (3.0, 0.0, -0.25, 0.0),   # strafe right
    (6.0, 0.0, 0.0, 0.6),     # spin left
    (6.0, 0.0, 0.0, -0.6),    # spin right
    (3.0, 0.2, 0.15, 0.0),    # diagonal
    (3.0, -0.2, -0.15, 0.0),
    (8.0, 0.15, 0.0, 0.4),    # arc
    (8.0, -0.15, 0.0, -0.4),
    (2.0, 0.0, 0.0, 0.0),
]


def main():
    rospy.init_node("drive_test")
    loops = rospy.get_param("~loops", 1)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.sleep(1.0)
    rate = rospy.Rate(20)
    for loop in range(loops):
        rospy.loginfo("drive_test: loop %d/%d", loop + 1, loops)
        for duration, vx, vy, wz in PATTERN:
            end = rospy.Time.now() + rospy.Duration(duration)
            while not rospy.is_shutdown() and rospy.Time.now() < end:
                cmd = Twist()
                cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
                pub.publish(cmd)
                rate.sleep()
    pub.publish(Twist())
    rospy.loginfo("drive_test: done")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
