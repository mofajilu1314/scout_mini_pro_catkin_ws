# Third-party notice

## TD3 (network + replay + update)

- Source: [sfujim/TD3](https://github.com/sfujim/TD3) (Scott Fujimoto)
- Paper: Fujimoto et al., 2018 — https://arxiv.org/abs/1802.09477
- License: MIT — see `LICENSE_sfujim_TD3`
- Adaptation for Scout: vector `max_action` for `(v, ω)`; ROS Gazebo bridge in sibling modules

## Training-loop structure (ROS / Gazebo)

- Inspired by AUV+TD3 ROS Noetic patterns described in
  [DRL-AUV-navigation](https://github.com/lanyangyang-gogogo/DRL-AUV-navigation)
  (upstream currently README-only; we did **not** copy AUV dynamics)
- Hierarchical local-goal idea deferred to stage B/C:
  [Arena-Rosnav](https://github.com/ignc-research/arena-rosnav) — **not** integrated yet
