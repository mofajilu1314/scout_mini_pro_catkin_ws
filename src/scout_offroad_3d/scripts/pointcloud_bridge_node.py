#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo block_laser publishes sensor_msgs/PointCloud; republish as PointCloud2 for RViz/tools.
"""

from __future__ import print_function

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from sensor_msgs import point_cloud2


def _cloud_to_cloud2(cloud):
    points = []
    for p in cloud.points:
        if (
            abs(p.x) < 1e-6
            and abs(p.y) < 1e-6
            and abs(p.z) < 1e-6
        ):
            continue
        points.append((p.x, p.y, p.z))
    header = cloud.header
    return point_cloud2.create_cloud_xyz32(header, points)


class PointCloudBridge(object):
    def __init__(self):
        in_topic = rospy.get_param("~in_topic", "/lidar/points")
        out_topic = rospy.get_param("~out_topic", "/lidar/points2")
        self._pub = rospy.Publisher(out_topic, PointCloud2, queue_size=1)
        self._sub = rospy.Subscriber(in_topic, PointCloud, self._on_cloud, queue_size=1)
        rospy.loginfo(
            "pointcloud_bridge: %s (PointCloud) -> %s (PointCloud2)",
            in_topic,
            out_topic,
        )

    def _on_cloud(self, msg):
        self._pub.publish(_cloud_to_cloud2(msg))


def main():
    rospy.init_node("pointcloud_bridge_node")
    PointCloudBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
