"""Filesystem-only tests for the TD3 training-run safety contract."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from scout_offroad_3d.training_run_contract import (  # noqa: E402
    REQUIRED_RESUME_ARTIFACTS,
    create_unique_run_directory,
    fresh_output_error,
    missing_resume_artifacts,
    resolve_workspace_root,
    resume_horizon_error,
    snapshot_dimension_errors,
    workspace_root_from_package_share,
)
from scout_offroad_3d.train_td3_node import (  # noqa: E402
    CURRENT_TRAINING_RECORD,
    _save_run_config,
    _write_bootstrap_config,
)


class TrainingRunContractTest(unittest.TestCase):
    def test_fresh_output_rejects_any_existing_content(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            output = root / "output"
            self.assertIsNone(fresh_output_error(str(output)))
            output.mkdir()
            self.assertIsNone(fresh_output_error(str(output)))
            (output / "previous_metrics.csv").write_text("old run\n")
            self.assertEqual(
                fresh_output_error(str(output)), "output directory is not empty"
            )

    def test_fresh_output_rejects_a_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "not_a_directory"
            output.write_text("not a run directory\n")
            self.assertEqual(
                fresh_output_error(str(output)),
                "output path exists but is not a directory",
            )

    def test_unique_runs_claim_distinct_directories(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs_dir = Path(tmp) / "runs"
            first = create_unique_run_directory(
                str(runs_dir), timestamp="20300102_030405"
            )
            second = create_unique_run_directory(
                str(runs_dir), timestamp="20300102_030405"
            )
            self.assertEqual(Path(first).name, "run_20300102_030405")
            self.assertEqual(Path(second).name, "run_20300102_030405_01")
            self.assertTrue(Path(first).is_dir())
            self.assertTrue(Path(second).is_dir())

    def test_resume_requires_every_nonempty_artifact(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            self.assertEqual(
                set(missing_resume_artifacts(str(run_dir))),
                set(REQUIRED_RESUME_ARTIFACTS),
            )
            for name in REQUIRED_RESUME_ARTIFACTS:
                (run_dir / name).write_text("present\n")
            self.assertEqual(missing_resume_artifacts(str(run_dir)), ())
            (run_dir / "td3_actor_optimizer").write_text("")
            self.assertEqual(
                missing_resume_artifacts(str(run_dir)), ("td3_actor_optimizer",)
            )

    def test_resume_horizon_must_be_explicit_and_forward_only(self):
        self.assertEqual(
            resume_horizon_error(False, 100, 99),
            "resume requires an explicit max_timesteps:=N launch argument",
        )
        self.assertEqual(
            resume_horizon_error(True, 100, 100),
            "max_timesteps=100 must be greater than completed t=100",
        )
        self.assertIsNone(resume_horizon_error(True, 101, 100))

    def test_snapshot_dimension_metadata_must_agree(self):
        valid = {
            "meta": {"state_dim": 150, "action_dim": 2},
            "train": {
                "hidden_dim": 1024,
                "kwargs": {"state_dim": 150, "action_dim": 2, "hidden_dim": 1024},
            },
        }
        self.assertEqual(snapshot_dimension_errors(valid), ())
        invalid = {
            "meta": {"state_dim": 150, "action_dim": 2},
            "train": {
                "hidden_dim": 1024,
                "kwargs": {"state_dim": 149, "action_dim": 2, "hidden_dim": 768},
            },
        }
        self.assertEqual(
            snapshot_dimension_errors(invalid),
            (
                "train.kwargs.state_dim=149 does not match snapshot 150",
                "train.kwargs.hidden_dim=768 does not match snapshot 1024",
            ),
        )

    def test_workspace_resolution_has_fixed_priority_then_package(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "scout_workspace"
            package_share = (
                root
                / "ros2_ws"
                / "src"
                / "navigation_3d"
                / "scout_offroad_3d"
            )
            package_share.mkdir(parents=True)
            (root / "3D").mkdir()
            self.assertEqual(
                resolve_workspace_root(
                    {
                        "SCOUT_WS_ROOT": "/opt/scout",
                        "CATKIN_WS": "/opt/catkin",
                        "WS_ROOT": "/opt/ws",
                    },
                    str(package_share),
                ),
                "/opt/scout",
            )
            self.assertEqual(
                resolve_workspace_root(
                    {"CATKIN_WS": "/opt/catkin", "WS_ROOT": "/opt/ws"}
                ),
                "/opt/catkin",
            )
            self.assertEqual(resolve_workspace_root({"WS_ROOT": "/opt/ws"}), "/opt/ws")
            self.assertEqual(
                resolve_workspace_root({}, str(package_share)), str(root)
            )
            self.assertEqual(
                workspace_root_from_package_share(str(package_share)), str(root)
            )

    def test_unrecognised_package_path_has_no_machine_specific_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            package_share = Path(tmp) / "unrelated" / "share" / "scout_offroad_3d"
            package_share.mkdir(parents=True)
            self.assertIsNone(resolve_workspace_root({}, str(package_share)))

    def test_new_run_metadata_uses_the_current_record(self):
        class Env:
            current_world = "/tmp/bumpy_offroad.world"
            state_dim = 150
            action_dim = 2
            max_action = np.array([2.3, 1.8])

            @staticmethod
            def snapshot_config():
                return {"state_dim": 150, "action_dim": 2}

        with tempfile.TemporaryDirectory() as tmp:
            out_root = Path(tmp)
            train_cfg = {"hidden_dim": 1024}
            _write_bootstrap_config(str(out_root), train_cfg, "/workspace")
            bootstrap = yaml.safe_load((out_root / "config.yaml").read_text())
            self.assertEqual(bootstrap["meta"]["record"], CURRENT_TRAINING_RECORD)
            self.assertEqual(CURRENT_TRAINING_RECORD, 9)

            _save_run_config(str(out_root), Env(), train_cfg, "/workspace")
            for filename in ("config.yaml", "params_backup.yaml"):
                snapshot = yaml.safe_load((out_root / filename).read_text())
                self.assertEqual(snapshot["meta"]["record"], 9)


if __name__ == "__main__":
    unittest.main()
