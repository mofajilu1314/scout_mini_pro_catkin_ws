#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish odom → base_link TF from /odom, skipping non-increasing stamps.

gazebo_ros_skid_steer_drive with broadcastTF=true uses the old tf API and can
emit redundant timestamps (TF_REPEATED_DATA / unknown_publisher). Keep the plugin
on /odom only; this node is the single TF authority for odom→base_link.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomToTfOfficial(object):
    def __init__(self):
        self._last_stamp = rospy.Time(0)
        self._br = tf2_ros.TransformBroadcaster()
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        rospy.Subscriber(odom_topic, Odometry, self._cb, queue_size=1)

    def _cb(self, msg):
        stamp = msg.header.stamp
        if stamp <= self._last_stamp:
            return
        self._last_stamp = stamp

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rospy.init_node("odom_to_tf_official")
    OdomToTfOfficial()
    rospy.spin()


if __name__ == "__main__":
    main()
