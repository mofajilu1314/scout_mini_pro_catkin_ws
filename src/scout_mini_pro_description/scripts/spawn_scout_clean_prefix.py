#!/usr/bin/env python3
"""roslaunch launch-prefix: delete embedded scout_mini, then exec spawn_model."""

from __future__ import print_function

import os
import sys
import time

import rospy
from gazebo_msgs.srv import DeleteModel


def _delete_model(model_name):
    rospy.wait_for_service("/gazebo/delete_model", timeout=10.0)
    delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    resp = delete(model_name)
    return bool(resp.success), getattr(resp, "status_message", "")


def main():
    delay = float(os.environ.get("SPAWN_DELAY", "1"))
    model = os.environ.get("SPAWN_MODEL_NAME", "scout_mini")

    if len(sys.argv) < 2:
        print("usage: spawn_scout_clean_prefix.py <spawn_model> [args...]", file=sys.stderr)
        sys.exit(2)

    rospy.init_node("spawn_scout_clean_prefix", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=120.0)
    time.sleep(delay)

    for attempt in range(8):
        try:
            ok, msg = _delete_model(model)
            rospy.loginfo(
                "delete_model(%s) attempt %d: ok=%s msg=%s",
                model,
                attempt + 1,
                ok,
                msg,
            )
            if ok:
                break
            # First launch / clean world: nothing to delete — skip remaining retries.
            if msg and "does not exist" in msg.lower():
                break
        except Exception as exc:
            rospy.logwarn("delete_model(%s) attempt %d failed: %s", model, attempt + 1, exc)
        time.sleep(0.5)

    time.sleep(0.3)
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == "__main__":
    main()
