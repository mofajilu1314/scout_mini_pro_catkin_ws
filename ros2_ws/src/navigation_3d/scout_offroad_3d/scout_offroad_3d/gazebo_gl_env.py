"""Portable NVIDIA PRIME / GLX env for Gazebo Classic (gzserver + gzclient).

Does not hard-code GPU SKUs (5070 / 4090 / …). Headless ``gui:=false``
still needs these vars when using ``gpu_ray`` (and depth cameras): Gazebo
Classic ray-on-GPU goes through NVIDIA GL/EGL, not just gzclient.
"""

from __future__ import annotations

import os
import subprocess
from typing import Dict, MutableMapping, Optional

# PRIME render-offload: discrete NVIDIA for OpenGL/Vulkan clients on hybrid laptops.
_NVIDIA_GL_VARS: Dict[str, str] = {
    "__NV_PRIME_RENDER_OFFLOAD": "1",
    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
    "__VK_LAYER_NV_optimus": "NVIDIA_only",
}

# Launch / train world-switch share this so restart matches initial launch.
SCOUT_NVIDIA_GL_ENV = "SCOUT_NVIDIA_GL"


def _nvidia_present() -> bool:
    if os.path.exists("/dev/nvidia0"):
        return True
    try:
        r = subprocess.run(
            ["nvidia-smi"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
            check=False,
        )
        return r.returncode == 0
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return False


def nvidia_gl_environ(mode: str = "auto") -> Dict[str, str]:
    """Return env vars that force OpenGL/Vulkan onto NVIDIA, or ``{}``.

    Parameters
    ----------
    mode:
        ``auto`` — inject only if ``/dev/nvidia0`` exists or ``nvidia-smi`` runs;
        ``true`` / ``1`` / ``yes`` / ``on`` — always inject;
        ``false`` / ``0`` / ``no`` / ``off`` — never inject.
    """
    m = (mode or "auto").strip().lower()
    if m in ("false", "0", "no", "off"):
        return {}
    if m in ("true", "1", "yes", "on"):
        return dict(_NVIDIA_GL_VARS)
    # auto (default) and any unrecognized value → detect
    if _nvidia_present():
        return dict(_NVIDIA_GL_VARS)
    return {}


def merge_nvidia_gl_env(
    env: Optional[MutableMapping[str, str]] = None,
    mode: Optional[str] = None,
) -> MutableMapping[str, str]:
    """Merge ``nvidia_gl_environ`` into *env* (default: copy of ``os.environ``).

    Mode resolution: explicit *mode* → ``SCOUT_NVIDIA_GL`` → ``auto``.
    """
    out: MutableMapping[str, str] = os.environ.copy() if env is None else env
    if mode is None:
        mode = out.get(SCOUT_NVIDIA_GL_ENV, "auto")
    out[SCOUT_NVIDIA_GL_ENV] = (mode or "auto").strip().lower() or "auto"
    out.update(nvidia_gl_environ(out[SCOUT_NVIDIA_GL_ENV]))
    return out
