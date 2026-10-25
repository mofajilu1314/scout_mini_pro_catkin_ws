"""模型加载与校验。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


class ModelLoader:
    def __init__(self, config_path: Optional[Path] = None) -> None:
        if config_path is None:
            config_path = Path(__file__).parent.parent / "config" / "models.yaml"
        self._config_path = config_path
        self._models: List[Dict[str, Any]] = []
        self._loaded: Optional[Dict[str, Any]] = None
        self._load_registry()

    def _load_registry(self) -> None:
        if not self._config_path.exists():
            return
        with open(self._config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        self._models = data.get("models", [])

    def list_models(self) -> List[Dict[str, Any]]:
        return self._models

    def load(self, model_path: str) -> Tuple[bool, str]:
        """校验并加载模型目录。"""
        path = Path(model_path)
        if not path.is_absolute():
            path = self._config_path.parent / path
        path = path.resolve()

        if not path.exists():
            return False, f"路径不存在: {path}"

        # TD3.load() reads the complete prefix even for inference; accepting an
        # actor alone would let the UI report a model as usable when launch
        # fails later.
        required_weights = (
            "td3_actor",
            "td3_actor_optimizer",
            "td3_critic",
            "td3_critic_optimizer",
        )
        missing_weights = [name for name in required_weights if not (path / name).is_file()]
        if missing_weights:
            return False, "缺少 TD3 权重文件: " + ", ".join(missing_weights)

        # 读 config
        cfg = self._read_config(path)
        if cfg is None:
            return False, "无法读取 config.yaml / params_backup.yaml"

        # 校验维度
        state_dim = cfg.get("state_dim")
        hidden_dim = cfg.get("hidden_dim") or cfg.get("kwargs", {}).get("hidden_dim")
        if state_dim is None or hidden_dim is None:
            return False, "config 缺少 state_dim/hidden_dim"

        # 与注册表匹配
        matched = None
        for m in self._models:
            if m["state_dim"] == state_dim and m["hidden_dim"] == hidden_dim:
                matched = m
                break
        if matched is None:
            return False, f"未知模型结构: state_dim={state_dim}, hidden_dim={hidden_dim}"

        self._loaded = {
            "path": str(path),
            "name": matched.get("name", path.name),
            "state_dim": state_dim,
            "hidden_dim": hidden_dim,
            "n_sectors": matched.get("n_sectors", 36),
            "n_elevation_bins_td3": matched.get("n_elevation_bins_td3", 3),
            "use_imu_in_state": matched.get("use_imu_in_state", False),
            # A registry safety classification must travel with the loaded
            # model; otherwise a simulation-only checkpoint could be handed
            # from the shared selector to the real-vehicle panel.
            "simulation_only": bool(matched.get("simulation_only", False)),
        }
        return True, "ok"

    def _read_config(self, path: Path) -> Optional[Dict[str, Any]]:
        """读取配置，支持多种结构。优先读 params_backup.yaml（完整），config.yaml 可能只有 meta。"""
        # 优先 params_backup.yaml，因为它有完整的 meta/train/env
        for name in ("params_backup.yaml", "config.yaml"):
            p = path / name
            if p.exists():
                try:
                    with open(p, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f) or {}
                    # 扁平化：从 meta/train/kwargs/env 中提取
                    result = {}
                    if "meta" in data:
                        result["state_dim"] = data["meta"].get("state_dim")
                    if "train" in data:
                        result["hidden_dim"] = data["train"].get("hidden_dim")
                    if "kwargs" in data:
                        result["hidden_dim"] = result.get("hidden_dim") or data["kwargs"].get("hidden_dim")
                    if "env" in data:
                        result["state_dim"] = result.get("state_dim") or data["env"].get("state_dim")
                    # 也检查顶层
                    result["state_dim"] = result.get("state_dim") or data.get("state_dim")
                    result["hidden_dim"] = result.get("hidden_dim") or data.get("hidden_dim")
                    # 只有找到有效值才返回
                    if result.get("state_dim") is not None and result.get("hidden_dim") is not None:
                        return result
                except Exception:
                    continue
        return None

    @property
    def loaded(self) -> Optional[Dict[str, Any]]:
        return self._loaded
