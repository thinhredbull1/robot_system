#!/usr/bin/env python3

"""
Audio trigger node.

Watches the moving_server action and, when a MODE_MOVE_TO_POINT goal
succeeds, calls the delivery API (POST .../deliveries/completions) over
wifi. The server side triggers the audio.

moving_server itself is untouched: the node only listens to the
/moving_server/move_command goal and result topics. Docking reuses
move_base internally but is a different mode, so it never triggers the
call.
"""

import json
import ssl
import threading
import urllib.request

import rospy
from actionlib_msgs.msg import GoalStatus

from moving_server_msgs.msg import (
    MoveCommandActionGoal,
    MoveCommandActionResult,
    MoveCommandGoal,
)


class AudioNode:

    def __init__(self):

        self.url = rospy.get_param(
            "~url",
            "https://34.172.236.168:8081/api/v1/robots/MEDIBOT-01"
            "/deliveries/completions"
        )
        self.timeout = float(rospy.get_param("~timeout", 5.0))
        self.min_interval = float(rospy.get_param("~min_interval", 2.0))
        self.auth_token = rospy.get_param("~auth_token", "")

        # JSON body of the POST (a dict param), empty object by default
        self.payload = rospy.get_param("~payload", {})

        self.ssl_context = None

        if not rospy.get_param("~verify_ssl", True):

            # Self signed server certificate
            self.ssl_context = ssl._create_unverified_context()

        # goal id -> mode, to know what a result belongs to
        self.modes = {}

        self.last_sent = rospy.Time(0)
        self.lock = threading.Lock()

        ns = rospy.get_param("~action_ns", "/moving_server/move_command")

        rospy.Subscriber(
            ns + "/goal", MoveCommandActionGoal, self.on_goal,
            queue_size=10
        )
        rospy.Subscriber(
            ns + "/result", MoveCommandActionResult, self.on_result,
            queue_size=10
        )

        rospy.loginfo("[audio_node] ready, target %s", self.url)

    def on_goal(self, msg):

        self.modes[msg.goal_id.id] = msg.goal.mode

        # Bound the memory of a long running node
        while len(self.modes) > 50:
            self.modes.pop(next(iter(self.modes)))

    def on_result(self, msg):

        mode = self.modes.pop(msg.status.goal_id.id, None)

        if (
            mode != MoveCommandGoal.MODE_MOVE_TO_POINT
            or msg.status.status != GoalStatus.SUCCEEDED
            or not msg.result.success
        ):
            return

        now = rospy.Time.now()

        with self.lock:

            if (now - self.last_sent).to_sec() < self.min_interval:
                return

            self.last_sent = now

        # Off the callback thread: a slow server must not block ROS
        threading.Thread(target=self.send, daemon=True).start()

    def send(self):

        headers = {"Content-Type": "application/json"}

        if self.auth_token:
            headers["Authorization"] = "Bearer " + self.auth_token

        request = urllib.request.Request(
            self.url,
            data=json.dumps(self.payload).encode(),
            headers=headers,
            method="POST"
        )

        try:

            with urllib.request.urlopen(
                request, timeout=self.timeout, context=self.ssl_context
            ) as response:

                rospy.loginfo(
                    "[audio_node] completion sent, HTTP %d", response.status
                )

        except Exception as error:

            rospy.logerr("[audio_node] completion request failed: %s", error)


if __name__ == "__main__":

    rospy.init_node("audio_node")

    AudioNode()

    rospy.spin()
