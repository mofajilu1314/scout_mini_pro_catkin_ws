"""Filesystem-only checks for the initial Gazebo ownership handoff guard."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from scout_offroad_3d.td3.gazebo_ownership import (  # noqa: E402
    handoff_gzserver_identity,
    listener_pids_from_ss,
)


def _write_fake_process(root: Path, *, token: str, argv: list[str]) -> int:
    pid = 4242
    proc = root / str(pid)
    proc.mkdir()
    (proc / "cmdline").write_bytes(b"\0".join(x.encode() for x in argv) + b"\0")
    (proc / "environ").write_bytes(
        b"PATH=/usr/bin\0SCOUT_GAZEBO_OWNER_TOKEN="
        + token.encode()
        + b"\0"
    )
    # /proc/<pid>/stat fields after ')' start at field 3; starttime is field 22.
    fields = ["S"] + ["0"] * 18 + ["778899"] + ["0"]
    (proc / "stat").write_text("4242 (gzserver) " + " ".join(fields))
    return pid


class GazeboOwnerHandoffTest(unittest.TestCase):
    def test_listener_pid_parser_deduplicates_ipv4_ipv6_entries(self):
        details = (
            'LISTEN users:(("gzserver",pid=42,fd=7))\n'
            'LISTEN users:(("gzserver",pid=42,fd=8),("other",pid=9,fd=1))'
        )
        self.assertEqual(listener_pids_from_ss(details), [9, 42])

    def test_identity_requires_exact_token_gzserver_and_starttime(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pid = _write_fake_process(
                root,
                token="handoff-token",
                argv=["/usr/bin/gzserver", "world.world"],
            )
            self.assertEqual(
                handoff_gzserver_identity(pid, "handoff-token", str(root)),
                (pid, "778899"),
            )
            self.assertIsNone(
                handoff_gzserver_identity(pid, "different-token", str(root))
            )
            (root / str(pid) / "cmdline").write_bytes(b"/usr/bin/python3\0")
            self.assertIsNone(
                handoff_gzserver_identity(pid, "handoff-token", str(root))
            )


if __name__ == "__main__":
    unittest.main()
