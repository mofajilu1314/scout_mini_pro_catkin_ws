"""数据记录：本机 rosbag（仿真）/ 车上 SSH rosbag（实车）。"""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path
from typing import List, Optional

DEFAULT_RECORD_TOPICS = [
    "/cmd_vel",
    "/odom",
    "/offroad/train_goal",
]
OPTIONAL_RECORD_TOPICS = [
    "/rslidar_points",
    "/offroad/td3_range_features",
]
PRIVATE_REMOTE_BAG_DIR_ENV = "SCOUT_ROBOT_BAG_DIR"


def record_topics(include_lidar: bool = False, include_features: bool = False) -> List[str]:
    topics = list(DEFAULT_RECORD_TOPICS)
    if include_lidar:
        topics.append("/rslidar_points")
    if include_features or include_lidar:
        if "/offroad/td3_range_features" not in topics:
            topics.append("/offroad/td3_range_features")
    return topics


def resolve_remote_bag_dir(remote_dir: Optional[str] = None) -> str:
    """Return an explicitly supplied or privately configured remote bag directory.

    Public source must not select a vehicle account or directory.  A caller may
    supply the absolute vehicle-side directory directly, or set
    ``SCOUT_ROBOT_BAG_DIR`` in a local private deployment configuration.
    """
    candidate = (
        remote_dir
        if remote_dir is not None
        else os.environ.get(PRIVATE_REMOTE_BAG_DIR_ENV, "")
    )
    directory = (candidate or "").strip().rstrip("/")
    if not directory:
        raise ValueError(
            "未配置车辆端记录目录。请显式提供目录，或在本机私有配置中设置 "
            f"{PRIVATE_REMOTE_BAG_DIR_ENV}。"
        )
    if not directory.startswith("/"):
        raise ValueError("车辆端记录目录必须是绝对路径。")
    return directory


def ssh_start_record_cmd(
    include_lidar: bool = False, remote_dir: Optional[str] = None
) -> str:
    """Build a rosbag command only after a private remote directory is supplied."""
    bag_dir = shlex.quote(resolve_remote_bag_dir(remote_dir))
    topic_s = " ".join(record_topics(include_lidar=include_lidar))
    return (
        f"mkdir -p {bag_dir} && cd {bag_dir} && "
        f"nohup rosbag record -O scout_$(date +%Y%m%d_%H%M%S) {topic_s} "
        "> /tmp/scout_nav_bag.log 2>&1 & echo $!"
    )


class DataRecorder:
    def __init__(self, output_dir: Path) -> None:
        self._output_dir = output_dir
        self._process: Optional[subprocess.Popen] = None

    def start(self, topics: List[str], name: str = "session") -> bool:
        if self._process is not None:
            return False
        self._output_dir.mkdir(parents=True, exist_ok=True)
        out = self._output_dir / f"{name}.bag"
        cmd = ["rosbag", "record", "-O", str(out)] + topics
        self._process = subprocess.Popen(cmd)
        return True

    def stop(self) -> None:
        if self._process:
            self._process.terminate()
            self._process = None

    def is_recording(self) -> bool:
        return self._process is not None
