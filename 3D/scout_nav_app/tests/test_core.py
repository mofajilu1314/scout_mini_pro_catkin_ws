"""单元测试：模型加载与安全边界。"""

from __future__ import annotations

from scout_nav_app.model_loader import ModelLoader
from scout_nav_app.safety_boundary import SafetyBoundary
from scout_nav_app.coverage_utils import coverage_inside_boundary
from scout_nav_app.workspace_helper import workspace_file


def test_model_loader_registry() -> None:
    loader = ModelLoader()
    models = loader.list_models()
    assert len(models) >= 1
    assert any(m["name"] == "Record 9 (simulation only)" for m in models)
    assert all(m.get("simulation_only") is True for m in models)


def test_model_loader_bad_path() -> None:
    loader = ModelLoader()
    ok, msg = loader.load("/nonexistent/path")
    assert not ok
    assert "不存在" in msg


def test_record_9_load_keeps_simulation_only_guard() -> None:
    loader = ModelLoader()
    record_9 = next(
        model for model in loader.list_models() if model["name"] == "Record 9 (simulation only)"
    )
    ok, msg = loader.load(record_9["path"])
    assert ok, msg
    assert loader.loaded is not None
    assert loader.loaded["simulation_only"] is True


def test_safety_boundary() -> None:
    b = SafetyBoundary()
    ok, msg = b.add_point(0, 0)
    assert not ok and "未在设置" in msg
    b.begin_edit()
    assert b.add_point(0, 0)[0]
    assert b.add_point(1, 0)[0]
    assert b.add_point(1, 1)[0]
    assert not b.is_valid()
    ok, msg = b.close()
    assert ok and b.is_valid()
    assert len(b.points()) == 3
    ok, _ = b.undo_last()
    assert ok and not b.is_valid()
    b.reset()
    assert len(b.points()) == 0


def test_safety_boundary_self_intersect() -> None:
    b = SafetyBoundary()
    b.begin_edit()
    # bow-tie
    for x, y in [(0, 0), (1, 1), (1, 0), (0, 1)]:
        assert b.add_point(x, y)[0]
    ok, msg = b.close()
    assert not ok
    assert "交叉" in msg


def test_coverage_inside_full_map() -> None:
    map_dir = workspace_file("3D", "maps_long")
    # large box covering most of arena
    poly = [(-30, -30), (30, -30), (30, 30), (-30, 30)]
    ratio, info = coverage_inside_boundary(map_dir, poly, stride=4)
    assert info["total"] > 1000
    assert ratio > 0.5


def test_classify_connect_error_net_auth_timeout() -> None:
    from scout_nav_app.real_bridge import classify_connect_error

    k, m = classify_connect_error(False, None, "", False)
    assert k == "net" and "现场局域网" in m and "私有配置" in m
    k, m = classify_connect_error(True, None, "", True)
    assert k == "timeout"
    k, m = classify_connect_error(True, 255, "Permission denied (publickey)", False)
    assert k == "auth" and "密码" in m


def test_has_any_node_names() -> None:
    from scout_nav_app.real_bridge import _has_any

    text = "/scout_base_node\n/robot_state_publisher\n"
    assert _has_any(text, ("scout_base", "rslidar")) == ["scout_base"]
    assert not _has_any(text, ("rslidar",))


def test_same_prefix24_and_lan_hint() -> None:
    from scout_nav_app.real_bridge import same_prefix24

    assert same_prefix24("alpha.beta.gamma.one", "alpha.beta.gamma.two")
    assert not same_prefix24("left.one.two.three", "right.one.two.three")


def test_record_topics_and_ssh_cmd(monkeypatch) -> None:
    from scout_nav_app.data_recorder import record_topics, ssh_start_record_cmd

    base = record_topics()
    assert "/cmd_vel" in base and "/odom" in base and "/rslidar_points" not in base
    with_lidar = record_topics(include_lidar=True)
    assert "/rslidar_points" in with_lidar
    monkeypatch.delenv("SCOUT_ROBOT_BAG_DIR", raising=False)
    try:
        ssh_start_record_cmd(include_lidar=False)
    except ValueError as exc:
        assert "SCOUT_ROBOT_BAG_DIR" in str(exc)
    else:
        assert False, "missing private remote directory must refuse to build a command"
    monkeypatch.setenv("SCOUT_ROBOT_BAG_DIR", "/configured/recordings")
    cmd = ssh_start_record_cmd(include_lidar=False)
    assert "rosbag record" in cmd and "/cmd_vel" in cmd and "/configured/recordings" in cmd
    explicit_cmd = ssh_start_record_cmd(
        include_lidar=False, remote_dir="/configured/override"
    )
    assert "/configured/override" in explicit_cmd


def test_private_long_nav_source_requires_explicit_valid_package(monkeypatch, tmp_path) -> None:
    from scout_nav_app.real_bridge import RealBridge

    monkeypatch.delenv("SCOUT_ROBOT_LONG_NAV_SOURCE", raising=False)
    source, msg = RealBridge._private_long_nav_ros1_source()
    assert source is None and "SCOUT_ROBOT_LONG_NAV_SOURCE" in msg

    monkeypatch.setenv("SCOUT_ROBOT_LONG_NAV_SOURCE", "relative/package")
    source, msg = RealBridge._private_long_nav_ros1_source()
    assert source is None and "绝对路径" in msg

    package_dir = tmp_path / "scout_long_nav_ros1"
    package_dir.mkdir()
    (package_dir / "package.xml").write_text("<package/>", encoding="utf-8")
    monkeypatch.setenv("SCOUT_ROBOT_LONG_NAV_SOURCE", str(package_dir))
    source, msg = RealBridge._private_long_nav_ros1_source()
    assert source == package_dir and not msg


def test_parse_safety_sample_odom_and_feat() -> None:
    from scout_nav_app.real_bridge import parse_safety_sample

    raw = """
===LOG===
ok
===ODOM===
pose:
  pose:
    position:
      x: 1.25
      y: -0.5
      z: 0.0
===IMU===
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
===FEAT===
data: [3.2, 2.1, 4.0]
===END===
"""
    s = parse_safety_sample(raw)
    assert s["x"] == 1.25 and s["y"] == -0.5
    assert s["front"] == 2.1


def test_parse_battery_voltage_and_percent() -> None:
    from scout_nav_app.real_bridge import (
        format_battery_chip,
        parse_battery,
        voltage_to_pct,
    )

    assert voltage_to_pct(21.5) == 0
    assert voltage_to_pct(29.2) == 100
    assert voltage_to_pct(24.0) == 15
    v, p = parse_battery("Welcome to Ubuntu\nbattery_voltage: 26.4\n")
    assert v == 26.4 and p == voltage_to_pct(26.4)
    v, p = parse_battery("percentage: 0.72\nvoltage: 26.1\n")
    assert p == 72
    v, p = parse_battery("")
    assert v is None and p is None
    label, shown, tip = format_battery_chip(False, None, None)
    assert label == "未连接" and shown == -1
    label, shown, _ = format_battery_chip(True, 26.4, 72)
    assert label == "72%" and shown == 72
    label, shown, _ = format_battery_chip(True, None, None, pending=True)
    assert label == "…"
    label, shown, _ = format_battery_chip(True, None, None)
    assert label == "—"
