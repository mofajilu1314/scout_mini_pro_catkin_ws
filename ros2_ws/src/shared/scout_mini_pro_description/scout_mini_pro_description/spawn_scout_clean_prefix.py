#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Delete embedded scout_mini if present, then spawn URDF from robot_description."""

from __future__ import annotations

import math
import re
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_srvs.srv import Empty


def resolve_package_mesh_uris(urdf_xml: str) -> str:
    """Rewrite package://mesh URIs to absolute file:// paths.

    Gazebo Classic + Humble spawn_entity often leaves package:// unresolved, so
    visuals never load (only collision/inertia debug boxes show). ROS1 gazebo_ros
    usually expanded these; we do it explicitly before SpawnEntity.
    """

    def _repl(m: re.Match) -> str:
        pkg, rel = m.group(1), m.group(2)
        try:
            share = get_package_share_directory(pkg)
        except Exception:
            return m.group(0)
        path = share + "/" + rel
        return 'filename="file://' + path + '"'

    return re.sub(
        r'filename="package://([^/]+)/([^"]+)"',
        _repl,
        urdf_xml,
    )

def _wait_for_service(node: Node, client, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            return True
        rclpy.spin_once(node, timeout_sec=0.0)
    return False


def _model_exists(node: Node, model_name: str) -> bool:
    """Best-effort existence check via get_entity_state (avoids hung delete)."""
    from gazebo_msgs.srv import GetEntityState

    client = node.create_client(GetEntityState, "/get_entity_state")
    if not _wait_for_service(node, client, 10.0):
        return False
    req = GetEntityState.Request()
    req.name = model_name
    req.reference_frame = "world"
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        return False
    return bool(future.result().success)


def _delete_model(node: Node, model_name: str) -> tuple[bool, str]:
    client = node.create_client(DeleteEntity, "/delete_entity")
    if not _wait_for_service(node, client, 30.0):
        return False, "delete_entity service unavailable"
    req = DeleteEntity.Request()
    req.name = model_name
    future = client.call_async(req)
    # 短超时：实体不存在时 Gazebo 偶发不回包，别卡死 spawn
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        return False, "delete_entity call failed/timeout"
    resp = future.result()
    return bool(resp.success), str(resp.status_message)


def _unpause_physics(node: Node) -> tuple[bool, str]:
    client = node.create_client(Empty, "/unpause_physics")
    if not _wait_for_service(node, client, 30.0):
        return False, "unpause_physics service unavailable"
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done() or future.result() is None:
        return False, "unpause_physics call failed"
    return True, "ok"


def _spawn_urdf(
    node: Node,
    model_name: str,
    urdf_xml: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
) -> tuple[bool, str]:
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not _wait_for_service(node, client, 120.0):
        return False, "spawn_entity service unavailable"

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)

    req = SpawnEntity.Request()
    req.name = model_name
    req.xml = urdf_xml
    req.robot_namespace = "/"
    req.initial_pose = pose
    req.reference_frame = "world"

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
    if not future.done() or future.result() is None:
        return False, "spawn_entity call failed"
    resp = future.result()
    return bool(resp.success), str(resp.status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("spawn_scout_clean_prefix")

    node.declare_parameter("model_name", "scout_mini")
    node.declare_parameter("spawn_delay", 1.0)
    node.declare_parameter("x", 0.0)
    node.declare_parameter("y", 0.0)
    node.declare_parameter("z", 0.15)
    node.declare_parameter("yaw", 0.0)

    model_name = str(node.get_parameter("model_name").value)
    delay = float(node.get_parameter("spawn_delay").value)
    x = float(node.get_parameter("x").value)
    y = float(node.get_parameter("y").value)
    z = float(node.get_parameter("z").value)
    yaw = float(node.get_parameter("yaw").value)

    time.sleep(max(0.0, delay))

    exists = _model_exists(node, model_name)
    node.get_logger().info(f"pre-spawn get_entity_state({model_name}) exists={exists}")
    if exists:
        for attempt in range(4):
            ok, msg = _delete_model(node, model_name)
            node.get_logger().info(
                f"delete_entity({model_name}) attempt {attempt + 1}: ok={ok} msg={msg}"
            )
            if ok:
                break
            if msg and "does not exist" in msg.lower():
                break
            # 超时也继续 spawn，避免导航永远没有车
            if "timeout" in msg.lower():
                node.get_logger().warn("delete timed out — continue spawn anyway")
                break
            time.sleep(0.3)
    else:
        node.get_logger().info("model absent — skip delete_entity, spawn directly")

    time.sleep(0.3)

    if not node.has_parameter("robot_description"):
        node.declare_parameter("robot_description", "")
    urdf = str(node.get_parameter("robot_description").value).strip()
    if not urdf:
        node.get_logger().error("robot_description parameter is empty")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    urdf = resolve_package_mesh_uris(urdf)
    if "package://" in urdf:
        node.get_logger().warn(
            "robot_description still contains package:// after rewrite — "
            "some visuals may stay invisible in Gazebo"
        )
    else:
        node.get_logger().info("mesh URIs rewritten to file:// for Gazebo Classic")

    ok, msg = _spawn_urdf(node, model_name, urdf, x, y, z, yaw)
    if ok:
        node.get_logger().info(f"spawn OK: {msg}")
        up_ok, up_msg = _unpause_physics(node)
        node.get_logger().info(f"unpause_physics: ok={up_ok} msg={up_msg}")
        rc = 0
    else:
        node.get_logger().error(f"spawn failed: {msg}")
        rc = 1

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
