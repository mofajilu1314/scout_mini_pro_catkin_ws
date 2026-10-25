#!/usr/bin/env python3
"""Offline TD3 loss / objective landscape plots (static PNG + optional GIF).

Does not touch Gazebo or a running train process. Reads ``run_*/td3_*`` and
optional ``run_*/loss_landscape/ckpts/epXXXX/td3_*``.

Static outputs under ``run_*/loss_landscape/``:
  - critic_td_loss_3d.png
  - critic_td_loss_topo.png
  - actor_objective_3d.png

GIF (``--gif``): critic_td_loss_evol.gif when versioned ckpts exist.
"""

from __future__ import annotations

import argparse
import os
import re
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Config / TD3 load
# ---------------------------------------------------------------------------


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


def _load_run_config(run_dir):
    cfg_path = os.path.join(run_dir, "config.yaml")
    backup_path = os.path.join(run_dir, "params_backup.yaml")
    try:
        import yaml
    except ImportError as e:
        raise RuntimeError("PyYAML required: %s" % e)

    doc = {}
    for path in (cfg_path, backup_path):
        if not os.path.isfile(path):
            continue
        with open(path, "r") as f:
            loaded = yaml.safe_load(f) or {}
        if not isinstance(loaded, dict):
            continue
        # Shallow-merge: backup fills gaps left by a thin config.yaml
        for k, v in loaded.items():
            if k not in doc:
                doc[k] = v
            elif isinstance(doc.get(k), dict) and isinstance(v, dict):
                merged = dict(v)
                merged.update(doc[k])
                doc[k] = merged

    if not doc:
        raise FileNotFoundError("missing config.yaml / params_backup.yaml in %s" % run_dir)

    meta = doc.get("meta") or {}
    train = doc.get("train") or {}
    # ROS2 dump shape: /** / ros__parameters
    ros_params = {}
    star = doc.get("/**") or doc.get("*") or {}
    if isinstance(star, dict):
        ros_params = star.get("ros__parameters") or {}
    if not ros_params:
        ros_params = doc.get("ros__parameters") or {}

    nested = (doc.get("td3") or doc.get("policy") or {}) if isinstance(doc, dict) else {}

    def _pick(*keys, default=None):
        for src in (meta, train, ros_params, nested):
            if not isinstance(src, dict):
                continue
            for k in keys:
                if src.get(k) is not None:
                    return src.get(k)
        return default

    state_dim = int(_pick("state_dim", default=112))
    action_dim = int(_pick("action_dim", default=2))
    max_action = _pick("max_action", default=[2.3, 1.8])
    max_action = np.asarray(max_action, dtype=np.float32).reshape(-1)
    hidden_dim = int(_pick("hidden_dim", default=256))
    discount = float(_pick("discount", default=0.99))
    policy_noise = float(_pick("policy_noise", default=0.2))
    noise_clip = float(_pick("noise_clip", default=0.5))
    return {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "hidden_dim": hidden_dim,
        "discount": discount,
        "policy_noise": policy_noise,
        "noise_clip": noise_clip,
    }


def _resolve_hidden_dim(run_dir, cfg):
    """Prefer last hidden_dim in yaml dumps (train block often after meta)."""
    h = int(cfg["hidden_dim"])
    for name in ("params_backup.yaml", "config.yaml"):
        path = os.path.join(run_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, "r") as f:
                raw = f.read()
            found = re.findall(r"hidden_dim:\s*(\d+)", raw)
            if found:
                h = int(found[-1])
        except Exception:
            pass
    return h


def _build_policy(cfg):
    from scout_offroad_3d.td3.TD3 import TD3

    return TD3(
        state_dim=int(cfg["state_dim"]),
        action_dim=int(cfg["action_dim"]),
        max_action=cfg["max_action"],
        discount=float(cfg["discount"]),
        policy_noise=float(cfg["policy_noise"]),
        noise_clip=float(cfg["noise_clip"]),
        hidden_dim=int(cfg["hidden_dim"]),
    )


def _load_policy(prefix, cfg, map_location=None):
    """Load TD3 from ``prefix_{actor,critic}``; optimizers optional.

    Landscape ckpts from training save weights only (no ``*_optimizer``);
    full ``policy.load`` would fail on those.
    """
    import copy

    import torch

    policy = _build_policy(cfg)
    loc = map_location or ("cuda" if torch.cuda.is_available() else "cpu")
    actor_path = prefix + "_actor"
    critic_path = prefix + "_critic"
    if not os.path.isfile(actor_path) or not os.path.isfile(critic_path):
        raise FileNotFoundError("missing %s_actor/_critic" % prefix)

    has_opt = os.path.isfile(prefix + "_actor_optimizer") and os.path.isfile(
        prefix + "_critic_optimizer"
    )
    if has_opt:
        policy.load(prefix, map_location=loc)
    else:
        policy.critic.load_state_dict(torch.load(critic_path, map_location=loc))
        policy.actor.load_state_dict(torch.load(actor_path, map_location=loc))
        policy.critic_target = copy.deepcopy(policy.critic)
        policy.actor_target = copy.deepcopy(policy.actor)

    policy.actor.eval()
    policy.critic.eval()
    policy.actor_target.eval()
    policy.critic_target.eval()
    return policy


# ---------------------------------------------------------------------------
# Probe batch + random directions
# ---------------------------------------------------------------------------


def _make_probe_batch(cfg, batch_size=256, seed=0, probe_path=None):
    import torch

    if probe_path and os.path.isfile(probe_path):
        blob = torch.load(probe_path, map_location="cpu")
        return (
            blob["state"].float(),
            blob["action"].float(),
            blob["next_state"].float(),
            blob["reward"].float(),
            blob["not_done"].float(),
        )

    rng = np.random.RandomState(int(seed))
    sd, ad = int(cfg["state_dim"]), int(cfg["action_dim"])
    max_a = np.asarray(cfg["max_action"], dtype=np.float32).reshape(-1)
    if max_a.size == 1:
        max_a = np.full((ad,), float(max_a[0]), dtype=np.float32)

    state = rng.rand(batch_size, sd).astype(np.float32)
    if sd >= 4:
        state[:, -4] = rng.uniform(0.0, 1.0, size=batch_size).astype(np.float32)
        state[:, -3] = rng.uniform(-1.0, 1.0, size=batch_size).astype(np.float32)
        state[:, -2] = rng.uniform(-1.0, 1.0, size=batch_size).astype(np.float32)
        state[:, -1] = rng.uniform(-1.0, 1.0, size=batch_size).astype(np.float32)
    next_state = np.clip(
        state + rng.randn(batch_size, sd).astype(np.float32) * 0.05, 0.0, 1.0
    )
    if sd >= 4:
        next_state[:, -4:] = np.clip(
            state[:, -4:] + rng.randn(batch_size, 4).astype(np.float32) * 0.05,
            -1.0,
            1.0,
        )
        next_state[:, -4] = np.clip(next_state[:, -4], 0.0, 1.0)
    action = (rng.rand(batch_size, ad).astype(np.float32) * 2.0 - 1.0) * max_a.reshape(
        1, -1
    )
    reward = rng.randn(batch_size, 1).astype(np.float32) * 0.5
    not_done = np.ones((batch_size, 1), dtype=np.float32)

    return (
        torch.FloatTensor(state),
        torch.FloatTensor(action),
        torch.FloatTensor(next_state),
        torch.FloatTensor(reward),
        torch.FloatTensor(not_done),
    )


def _filter_normalize_direction(model, rng):
    """Li et al. filter-wise normalized random direction matching ``model``."""
    import torch

    direction = {}
    for name, p in model.named_parameters():
        if not p.requires_grad:
            continue
        d = torch.as_tensor(rng.randn(*p.shape), dtype=p.dtype, device=p.device)
        if p.dim() <= 1:
            n_p = float(torch.norm(p).item()) + 1e-10
            n_d = float(torch.norm(d).item()) + 1e-10
            d = d * (n_p / n_d)
        else:
            for i in range(p.shape[0]):
                n_p = float(torch.norm(p[i]).item()) + 1e-10
                n_d = float(torch.norm(d[i]).item()) + 1e-10
                d[i].mul_(n_p / n_d)
        direction[name] = d
    return direction


def _snapshot_state(model):
    return {k: v.detach().clone() for k, v in model.state_dict().items()}


# ---------------------------------------------------------------------------
# Loss surfaces
# ---------------------------------------------------------------------------


def _critic_td_loss(policy, batch):
    import torch
    import torch.nn.functional as F

    state, action, next_state, reward, not_done = batch
    device = next(policy.critic.parameters()).device
    state = state.to(device)
    action = action.to(device)
    next_state = next_state.to(device)
    reward = reward.to(device)
    not_done = not_done.to(device)

    with torch.no_grad():
        noise = (
            torch.randn_like(action) * policy.policy_noise * policy.max_action
        ).clamp(
            -policy.noise_clip * policy.max_action,
            policy.noise_clip * policy.max_action,
        )
        next_action = (policy.actor_target(next_state) + noise).clamp(
            -policy.max_action, policy.max_action
        )
        tq1, tq2 = policy.critic_target(next_state, next_action)
        target_q = reward + not_done * policy.discount * torch.min(tq1, tq2)

    q1, q2 = policy.critic(state, action)
    loss = F.mse_loss(q1, target_q) + F.mse_loss(q2, target_q)
    return float(loss.item())


def _actor_objective(policy, batch):
    """Mean -Q1(s, pi(s)); lower = better according to critic."""
    import torch

    state = batch[0].to(next(policy.actor.parameters()).device)
    with torch.no_grad():
        act = policy.actor(state)
        q = policy.critic.Q1(state, act)
        obj = (-q).mean()
    return float(obj.item())


def compute_surface(policy, which, batch, grid_steps=41, radius=1.0, seed=0):
    """Return alphas, betas, Z for critic|actor landscape."""
    import torch

    rng = np.random.RandomState(int(seed) + (1 if which == "actor" else 0))
    if which == "critic":
        model = policy.critic
        loss_fn = lambda: _critic_td_loss(policy, batch)
    elif which == "actor":
        model = policy.actor
        loss_fn = lambda: _actor_objective(policy, batch)
    else:
        raise ValueError("which must be critic|actor")

    base = _snapshot_state(model)
    d1 = _filter_normalize_direction(model, rng)
    d2 = _filter_normalize_direction(model, rng)

    alphas = np.linspace(-radius, radius, int(grid_steps))
    betas = np.linspace(-radius, radius, int(grid_steps))
    Z = np.zeros((len(betas), len(alphas)), dtype=np.float64)

    try:
        for i, b in enumerate(betas):
            for j, a in enumerate(alphas):
                with torch.no_grad():
                    for name, p in model.named_parameters():
                        if name not in d1:
                            continue
                        p.copy_(
                            base[name]
                            + float(a) * d1[name]
                            + float(b) * d2[name]
                        )
                Z[i, j] = loss_fn()
    finally:
        model.load_state_dict(base)

    return alphas, betas, Z


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def _plot_surface_3d(alphas, betas, Z, out_png, title, zlabel):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    A, B = np.meshgrid(alphas, betas)
    fig = plt.figure(figsize=(9.0, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#000000")
    ax.set_facecolor("#000000")
    surf = ax.plot_surface(
        A,
        B,
        Z,
        cmap="viridis",
        linewidth=0.15,
        antialiased=True,
        rstride=1,
        cstride=1,
        edgecolor=(0.2, 0.25, 0.2, 0.25),
    )
    ax.set_xlabel(r"$\alpha$", color="white")
    ax.set_ylabel(r"$\beta$", color="white")
    ax.set_zlabel(zlabel, color="white")
    ax.set_title(title, color="white", pad=10)
    ax.tick_params(colors="white")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#333333")
    ax.yaxis.pane.set_edgecolor("#333333")
    ax.zaxis.pane.set_edgecolor("#333333")
    cb = fig.colorbar(surf, ax=ax, shrink=0.55, pad=0.08)
    cb.ax.yaxis.set_tick_params(color="white")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="white")
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=160, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def _plot_topo(alphas, betas, Z, out_png, title):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    A, B = np.meshgrid(alphas, betas)
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    fig.patch.set_facecolor("white")
    cf = ax.contourf(A, B, Z, levels=24, cmap="viridis")
    ax.contour(A, B, Z, levels=12, colors="#222222", linewidths=0.35, alpha=0.45)
    ax.set_xlabel(r"$\alpha$")
    ax.set_ylabel(r"$\beta$")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.04, label="value")
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def render_static_for_prefix(
    prefix, cfg, out_dir, batch, grid_steps=41, radius=1.0, seed=0
):
    policy = _load_policy(prefix, cfg)
    written = []

    a, b, Zc = compute_surface(
        policy, "critic", batch, grid_steps=grid_steps, radius=radius, seed=seed
    )
    p3 = os.path.join(out_dir, "critic_td_loss_3d.png")
    pt = os.path.join(out_dir, "critic_td_loss_topo.png")
    _plot_surface_3d(
        a, b, Zc, p3, title="Critic TD-loss landscape", zlabel="TD loss"
    )
    _plot_topo(a, b, Zc, pt, title="Critic TD-loss (topo)")
    written.extend([p3, pt])

    a2, b2, Za = compute_surface(
        policy, "actor", batch, grid_steps=grid_steps, radius=radius, seed=seed
    )
    pa = os.path.join(out_dir, "actor_objective_3d.png")
    _plot_surface_3d(
        a2,
        b2,
        Za,
        pa,
        title="Actor objective landscape ($-Q$)",
        zlabel=r"$-Q_1$",
    )
    written.append(pa)
    return written, (a, b, Zc)


def _list_ckpts(run_dir):
    """List (sort_key, prefix) for GIF frames under loss_landscape/ckpts/*.

    Accepts EpisodeInterval tags ``ep0120`` and legacy ``periodic_ep250`` /
    ``sr10`` folders that still contain ``td3_{actor,critic}``.
    """
    root = os.path.join(run_dir, "loss_landscape", "ckpts")
    if not os.path.isdir(root):
        return []
    items = []
    for name in sorted(os.listdir(root)):
        prefix = os.path.join(root, name, "td3")
        if not (
            os.path.isfile(prefix + "_actor") and os.path.isfile(prefix + "_critic")
        ):
            continue
        m = re.match(r"ep(\d+)$", name)
        if m:
            key = int(m.group(1))
        else:
            m2 = re.search(r"ep(\d+)", name)
            key = int(m2.group(1)) if m2 else None
        if key is None:
            # srXX etc.: keep name order via stable secondary index
            key = 10 ** 9 + len(items)
        items.append((key, prefix, name))
    items.sort(key=lambda x: (x[0], x[2]))
    return [(k, p) for k, p, _n in items]


def render_gif(run_dir, cfg, batch, grid_steps=31, radius=1.0, seed=0):
    """Build critic_td_loss_evol.gif from versioned ckpts. Returns path or None."""
    ckpts = _list_ckpts(run_dir)
    out_dir = os.path.join(run_dir, "loss_landscape")
    _ensure_dir(out_dir)
    if not ckpts:
        print(
            "[landscape] no ckpts under %s/loss_landscape/ckpts/epXXXX/ "
            "(enable landscape_ckpt_enable on a future train)" % run_dir
        )
        return None

    frames_dir = os.path.join(out_dir, "_gif_frames")
    _ensure_dir(frames_dir)
    frame_paths = []
    for ep, prefix in ckpts:
        policy = _load_policy(prefix, cfg)
        a, b, Z = compute_surface(
            policy, "critic", batch, grid_steps=grid_steps, radius=radius, seed=seed
        )
        fp = os.path.join(frames_dir, "ep%04d.png" % ep)
        _plot_surface_3d(
            a,
            b,
            Z,
            fp,
            title="Critic TD-loss | ep%04d" % ep,
            zlabel="TD loss",
        )
        frame_paths.append(fp)
        print("[landscape] gif frame ep%04d → %s" % (ep, fp))

    gif_path = os.path.join(out_dir, "critic_td_loss_evol.gif")
    try:
        from PIL import Image

        imgs = [Image.open(p).convert("RGB") for p in frame_paths]
        imgs[0].save(
            gif_path,
            save_all=True,
            append_images=imgs[1:],
            duration=600,
            loop=0,
        )
    except Exception as e:
        print(
            "[landscape] GIF assemble failed (%s); frames kept in %s"
            % (e, frames_dir)
        )
        return None
    print("[landscape] GIF → %s" % gif_path)
    return gif_path


def render_landscape_for_run(
    run_dir,
    gif=True,
    grid_steps=41,
    gif_grid=31,
    radius=1.0,
    batch_size=256,
    seed=0,
    log_fn=None,
):
    """Generate static landscape PNGs (and GIF if ckpts exist). Returns written paths.

    Safe to call from train shutdown; does not raise on missing weights/ckpts.
    """
    def _log(msg, *a):
        if log_fn:
            try:
                log_fn(msg % a if a else msg)
            except Exception:
                print(msg % a if a else msg)
        else:
            print(msg % a if a else msg)

    try:
        from scout_offroad_3d.plot_style import apply_paper_style
    except Exception:
        try:
            from plot_style import apply_paper_style
        except Exception:
            apply_paper_style = None
    if apply_paper_style:
        apply_paper_style()

    run_dir = os.path.abspath(run_dir)
    written = []
    try:
        if not os.path.isdir(run_dir):
            _log("[landscape] skip: run_dir missing %s", run_dir)
            return written
        cfg = _load_run_config(run_dir)
        cfg["hidden_dim"] = _resolve_hidden_dim(run_dir, cfg)
        prefix = os.path.join(run_dir, "td3")
        if not os.path.isfile(prefix + "_actor"):
            _log("[landscape] skip: no %s_actor", prefix)
            return written
        out_dir = os.path.join(run_dir, "loss_landscape")
        _ensure_dir(out_dir)
        probe_path = os.path.join(out_dir, "landscape_probe.pt")
        batch = _make_probe_batch(
            cfg, batch_size=batch_size, seed=seed, probe_path=probe_path
        )
        _log(
            "[landscape] run=%s state_dim=%d hidden_dim=%d",
            run_dir,
            cfg["state_dim"],
            cfg["hidden_dim"],
        )
        paths, _ = render_static_for_prefix(
            prefix,
            cfg,
            out_dir,
            batch,
            grid_steps=grid_steps,
            radius=radius,
            seed=seed,
        )
        written.extend(paths)
        for p in paths:
            _log("[landscape] wrote %s", p)
        if gif:
            try:
                g = render_gif(
                    run_dir,
                    cfg,
                    batch,
                    grid_steps=gif_grid,
                    radius=radius,
                    seed=seed,
                )
                if g:
                    written.append(g)
                    _log("[landscape] GIF → %s", g)
                else:
                    _log("[landscape] GIF skipped (no usable ckpts or assemble fail)")
            except Exception as e_gif:
                _log("[landscape] GIF failed (static OK): %s", e_gif)
    except Exception as e:
        _log("[landscape] failed: %s", e)
    return written


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="TD3 loss / objective landscape (offline)"
    )
    parser.add_argument("run_dir", help="path to 3D/runs/run_YYYYMMDD_HHMMSS")
    parser.add_argument(
        "--gif", action="store_true", help="also build evol GIF if ckpts exist"
    )
    parser.add_argument("--grid", type=int, default=41, help="grid resolution (static)")
    parser.add_argument(
        "--gif-grid", type=int, default=31, help="grid resolution (GIF frames)"
    )
    parser.add_argument("--radius", type=float, default=1.0, help="α/β range")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--skip-static",
        action="store_true",
        help="only run --gif path (no static PNGs)",
    )
    args = parser.parse_args(argv)

    run_dir = os.path.abspath(args.run_dir)
    if not os.path.isdir(run_dir):
        print("run_dir not found: %s" % run_dir, file=sys.stderr)
        return 2

    if args.skip_static and args.gif:
        try:
            from scout_offroad_3d.plot_style import apply_paper_style
        except Exception:
            from plot_style import apply_paper_style

        apply_paper_style()
        cfg = _load_run_config(run_dir)
        cfg["hidden_dim"] = _resolve_hidden_dim(run_dir, cfg)
        out_dir = os.path.join(run_dir, "loss_landscape")
        _ensure_dir(out_dir)
        probe_path = os.path.join(out_dir, "landscape_probe.pt")
        batch = _make_probe_batch(
            cfg, batch_size=args.batch_size, seed=args.seed, probe_path=probe_path
        )
        render_gif(
            run_dir,
            cfg,
            batch,
            grid_steps=args.gif_grid,
            radius=args.radius,
            seed=args.seed,
        )
        return 0

    render_landscape_for_run(
        run_dir,
        gif=bool(args.gif),
        grid_steps=args.grid,
        gif_grid=args.gif_grid,
        radius=args.radius,
        batch_size=args.batch_size,
        seed=args.seed,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
