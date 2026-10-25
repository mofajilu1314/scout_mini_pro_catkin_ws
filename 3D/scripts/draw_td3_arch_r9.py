#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record 9 TD3 PPT parts: 2D squares / matrices, one asset per file.

No 3D cubes. Assemble in PowerPoint.

  python3 3D/scripts/draw_td3_arch_r9.py

Outputs: 3D/td3_arch_parts/*.png  (+ LOGIC.txt)
"""
from __future__ import annotations

import os
import textwrap

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "td3_arch_parts"))

N_I = 36  # horizontal sectors
N_J = 4   # elevation layers
STATE = 150
HIDDEN = 1024
ACTION = 2
BATCH = 1024


def _setup(w, h, dpi=200):
    fig, ax = plt.subplots(figsize=(w, h), dpi=dpi)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.0)
    return fig, ax


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=220, bbox_inches="tight", transparent=True, pad_inches=0.04)
    plt.close(fig)
    print("wrote", path)
    return path


def _cell(ax, x, y, s, text, fs=6.5, fc="#f7f7f7", ec="#222"):
    ax.add_patch(Rectangle((x, y), s, s, facecolor=fc, edgecolor=ec, lw=0.7, zorder=2))
    if text:
        ax.text(
            x + s / 2,
            y + s / 2,
            text,
            ha="center",
            va="center",
            fontsize=fs,
            zorder=3,
            family="DejaVu Sans",
        )


def part_L_matrix():
    """4 rows (j=layer) × 36 cols (i=sector): empty 2D squares, labels on axes."""
    s = 0.34
    gap = 0.05
    left, bottom = 1.15, 0.70
    w = left + N_I * (s + gap) + 0.55
    h = bottom + N_J * (s + gap) + 1.35
    fig, ax = _setup(w, h)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.text(
        w / 2,
        h - 0.28,
        r"$L_{i,j}$   $i=1\ldots 36$ horizontal sector    $j=1\ldots 4$ elevation layer",
        ha="center",
        fontsize=12,
        fontweight="bold",
    )
    ax.text(
        w / 2,
        h - 0.62,
        r"matrix $4\times 36=144$  ·  flatten as $[L_{\cdot,1};\,L_{\cdot,2};\,L_{\cdot,3};\,L_{\cdot,4}]$ into $s$",
        ha="center",
        fontsize=9,
        color="#444",
    )
    for j in range(N_J):
        jj = N_J - j
        y = bottom + j * (s + gap)
        ax.text(left - 0.18, y + s / 2, r"$j=%d$" % jj, ha="right", va="center", fontsize=11)
        for i in range(N_I):
            x = left + i * (s + gap)
            fc = "#93c5fd" if jj % 2 else "#dbeafe"
            _cell(ax, x, y, s, "", fc=fc)
        # one readable label per row, left cell
        ax.text(
            left + s / 2,
            y + s / 2,
            r"$L_{1,%d}$" % jj,
            ha="center",
            va="center",
            fontsize=6.5,
            zorder=4,
            color="#0f172a",
        )
    for i in (0, 5, 11, 17, 23, 29, 35):
        x = left + i * (s + gap) + s / 2
        ax.text(x, bottom - 0.28, str(i + 1), ha="center", fontsize=8, color="#333")
    ax.text(w / 2, 0.18, r"sector $i$", ha="center", fontsize=10)
    return _save(fig, "01_L_matrix.png")


def part_L_row():
    """One elevation layer as a row of squares: L_1,j ... L_36,j."""
    s = 0.42
    gap = 0.06
    left, bottom = 0.55, 0.55
    w = left + N_I * (s + gap) + 0.4
    h = 2.15
    fig, ax = _setup(w, h)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.text(
        w / 2,
        h - 0.28,
        r"one layer $j=1$:   $L_{1,1},\;L_{2,1},\;\ldots,\;L_{36,1}$",
        ha="center",
        fontsize=12,
        fontweight="bold",
    )
    y = bottom
    show = {0, 1, 2, 3, 34, 35}
    for i in range(N_I):
        x = left + i * (s + gap)
        lab = r"$L_{%d,1}$" % (i + 1) if i in show else (r"$\cdots$" if i == 4 else "")
        _cell(ax, x, y, s, lab, fs=7, fc="#dbeafe")
    ax.text(left + 17.5 * (s + gap), bottom - 0.32, r"$i=1\rightarrow 36$", ha="center", fontsize=9)
    return _save(fig, "01b_L_row.png")


def part_weight_matrix():
    """2D square grid for a weight matrix (not a cuboid)."""
    rows, cols = 8, 10
    s, gap = 0.38, 0.04
    left, bottom = 0.85, 0.55
    w = left + cols * (s + gap) + 0.9
    h = bottom + rows * (s + gap) + 1.15
    fig, ax = _setup(w, h)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.text(w / 2, h - 0.28, r"FC weight  (example $W$: squares, not cubes)", ha="center", fontsize=11, fontweight="bold")
    ax.text(w / 2, h - 0.58, r"Actor first layer: $W_1\in\mathbb{R}^{1024\times 150}$  (grid is schematic)", ha="center", fontsize=8.5, color="#444")
    for r in range(rows):
        for c in range(cols):
            x = left + c * (s + gap)
            y = bottom + (rows - 1 - r) * (s + gap)
            lab = ""
            if (r, c) == (0, 0):
                lab = r"$W_{11}$"
            elif (r, c) == (0, cols - 1):
                lab = r"$W_{1,n}$"
            elif (r, c) == (rows - 1, 0):
                lab = r"$W_{m1}$"
            elif r == 3 and c == 4:
                lab = r"$\cdots$"
            _cell(ax, x, y, s, lab, fs=7, fc="#e0f2fe")
    ax.text(left - 0.12, bottom + rows * (s + gap) / 2, r"$1024$", ha="right", va="center", fontsize=9, rotation=90)
    ax.text(left + cols * (s + gap) / 2, 0.18, r"$150$", ha="center", fontsize=9)
    return _save(fig, "08b_W_matrix.png")


def part_neuron_grid():
    """Hidden layer as a 2D array of squares (1024 units, shown as a patch)."""
    rows, cols = 8, 8
    s, gap = 0.36, 0.04
    left, bottom = 0.45, 0.45
    w = left + cols * (s + gap) + 0.45
    h = bottom + rows * (s + gap) + 1.05
    fig, ax = _setup(w, h)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.text(w / 2, h - 0.28, r"hidden  $h\in\mathbb{R}^{1024}$", ha="center", fontsize=12, fontweight="bold")
    ax.text(w / 2, h - 0.58, "2D squares (ellipsis = rest of 1024)", ha="center", fontsize=8, color="#555")
    for r in range(rows):
        for c in range(cols):
            x = left + c * (s + gap)
            y = bottom + (rows - 1 - r) * (s + gap)
            lab = r"$\cdots$" if r == 3 and c == 3 else ""
            _cell(ax, x, y, s, lab, fs=9, fc="#bfdbfe")
    return _save(fig, "08c_h1024_grid.png")


def part_column(symbols, title, fname, fc="#fef9c3"):
    n = len(symbols)
    s = 0.55
    pad = 0.35
    w = 2.4
    h = pad + n * s + 0.9
    fig, ax = _setup(w, h)
    ax.text(w / 2, h - 0.32, title, ha="center", fontsize=11, fontweight="bold")
    for k, sym in enumerate(symbols):
        y = pad + (n - 1 - k) * s
        _cell(ax, (w - s) / 2, y, s, sym, fs=11, fc=fc)
    return _save(fig, fname)


def part_s_stack():
    """One tall 2D column: show structure of s, not 150 tiny labels."""
    s = 0.42
    # groups: 8 lidar samples + dots + nav 4 + imu 2
    labels = [
        r"$L_{1,1}$",
        r"$L_{2,1}$",
        r"$\vdots$",
        r"$L_{36,4}$",
        r"$\rho$",
        r"$\theta$",
        r"$v$",
        r"$\omega$",
        r"roll$/\pi$",
        r"pitch$/\pi$",
    ]
    colors = (
        ["#dbeafe"] * 4
        + ["#fef9c3"] * 4
        + ["#fce7f3"] * 2
    )
    pad_t, pad_b = 1.05, 0.35
    w = 2.6
    h = pad_b + len(labels) * s + pad_t
    fig, ax = _setup(w, h)
    ax.text(w / 2, h - 0.28, r"state $s\in\mathbb{R}^{150}$", ha="center", fontsize=12, fontweight="bold")
    ax.text(w / 2, h - 0.58, "144 + 4 + 2  (column vector)", ha="center", fontsize=8, color="#555")
    x = (w - s) / 2
    for k, (lab, fc) in enumerate(zip(labels, colors)):
        y = pad_b + (len(labels) - 1 - k) * s
        _cell(ax, x, y, s, lab, fs=9, fc=fc)
    ax.text(x + s + 0.12, pad_b + 8.2 * s, "LiDAR", fontsize=8, color="#1e40af", va="center")
    ax.text(x + s + 0.12, pad_b + 4.5 * s, r"$\rho\theta vw$", fontsize=8, color="#a16207", va="center")
    ax.text(x + s + 0.12, pad_b + 0.5 * s, "IMU", fontsize=8, color="#9d174d", va="center")
    return _save(fig, "04_state_s_column.png")


def part_hidden(name, dim, fname, fc="#dbeafe"):
    s = 0.40
    labels = [r"$h_1$", r"$h_2$", r"$h_3$", r"$\vdots$", r"$h_{%d}$" % dim]
    pad = 0.45
    w = 2.2
    h = pad + len(labels) * s + 0.7
    fig, ax = _setup(w, h)
    ax.text(w / 2, h - 0.28, name, ha="center", fontsize=11, fontweight="bold")
    x = (w - s) / 2
    for k, lab in enumerate(labels):
        y = 0.25 + (len(labels) - 1 - k) * s
        _cell(ax, x, y, s, lab, fs=9, fc=fc)
    return _save(fig, fname)


def part_relu():
    fig, ax = plt.subplots(figsize=(3.2, 2.6), dpi=200)
    x = np.linspace(-3, 3, 200)
    y = np.maximum(0, x)
    ax.plot(x, y, color="#1d4ed8", lw=2.4)
    ax.axhline(0, color="#999", lw=0.6)
    ax.axvline(0, color="#999", lw=0.6)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-0.4, 3.1)
    ax.set_title(r"ReLU:  $\mathrm{ReLU}(z)=\max(0,z)$", fontsize=11)
    ax.set_xlabel(r"$z$", fontsize=10)
    ax.set_ylabel(r"$\mathrm{ReLU}(z)$", fontsize=10)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.patch.set_alpha(0.0)
    return _save(fig, "06_relu.png")


def part_tanh():
    fig, ax = plt.subplots(figsize=(3.2, 2.6), dpi=200)
    x = np.linspace(-3, 3, 200)
    y = np.tanh(x)
    ax.plot(x, y, color="#047857", lw=2.4)
    ax.axhline(0, color="#999", lw=0.6)
    ax.axvline(0, color="#999", lw=0.6)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_title(r"$\tanh$  (Actor output)", fontsize=11)
    ax.set_xlabel(r"$z$", fontsize=10)
    ax.set_ylabel(r"$\tanh(z)$", fontsize=10)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.patch.set_alpha(0.0)
    return _save(fig, "07_tanh.png")


def part_box(title, subtitle, fname, fc="#fff7ed", w=3.4, h=1.35):
    fig, ax = _setup(w, h)
    ax.add_patch(
        FancyBboxPatch(
            (0.08, 0.18),
            w - 0.16,
            h - 0.36,
            boxstyle="round,pad=0.04,rounding_size=0.08",
            facecolor=fc,
            edgecolor="#333",
            lw=1.2,
        )
    )
    ax.text(w / 2, h / 2 + 0.16, title, ha="center", va="center", fontsize=12, fontweight="bold")
    if subtitle:
        ax.text(w / 2, h / 2 - 0.22, subtitle, ha="center", va="center", fontsize=8.5, color="#444")
    return _save(fig, fname)


def part_arrow(fname, dx=1.6, dy=0.0):
    fig, ax = _setup(max(abs(dx) + 0.4, 1.2), max(abs(dy) + 0.4, 0.7))
    w = max(abs(dx) + 0.4, 1.2)
    h = max(abs(dy) + 0.4, 0.7)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    x0, y0 = 0.15, h / 2
    x1, y1 = w - 0.15, h / 2
    if abs(dy) > abs(dx):
        x0 = x1 = w / 2
        y0, y1 = h - 0.12, 0.12
    ax.annotate(
        "",
        xy=(x1, y1),
        xytext=(x0, y0),
        arrowprops=dict(arrowstyle="-|>", color="#111", lw=2.0, mutation_scale=16),
    )
    return _save(fig, fname)


def part_logic_sheet():
    """2D boxes + arrows only. No cubes."""
    fig, ax = _setup(12.4, 6.6)
    ax.set_xlim(0, 12.4)
    ax.set_ylim(0, 6.6)

    def box(x, y, ww, hh, title, sub, fc):
        ax.add_patch(
            FancyBboxPatch(
                (x, y),
                ww,
                hh,
                boxstyle="round,pad=0.03,rounding_size=0.06",
                facecolor=fc,
                edgecolor="#222",
                lw=1.0,
                zorder=2,
            )
        )
        ax.text(x + ww / 2, y + hh * 0.62, title, ha="center", fontsize=9.5, fontweight="bold", zorder=3)
        ax.text(x + ww / 2, y + hh * 0.28, sub, ha="center", fontsize=7.4, color="#333", zorder=3)

    def arr(a, b):
        ax.annotate(
            "",
            xy=b,
            xytext=a,
            arrowprops=dict(arrowstyle="-|>", color="#222", lw=1.15, mutation_scale=11),
            zorder=4,
        )

    ax.text(6.2, 6.28, "TD3 Record 9  —  what feeds what", ha="center", fontsize=13, fontweight="bold")

    box(0.25, 3.55, 1.7, 1.35, r"$L_{i,j}$", r"$36\times 4=144$", "#dbeafe")
    box(0.25, 2.05, 1.7, 1.15, r"$\rho,\theta,v,\omega$", "4", "#fef9c3")
    box(0.25, 0.70, 1.7, 1.05, r"roll, pitch", r"$/\pi$  →  2", "#fce7f3")
    box(2.20, 2.15, 1.45, 1.55, r"$s$", r"$\mathbb{R}^{150}$", "#e5e7eb")
    arr((1.95, 4.20), (2.20, 3.20))
    arr((1.95, 2.60), (2.20, 2.90))
    arr((1.95, 1.25), (2.20, 2.40))

    box(3.95, 3.85, 1.55, 1.05, "FC", r"$150\to 1024$", "#93c5fd")
    box(5.70, 3.85, 1.15, 1.05, "ReLU", "", "#bfdbfe")
    box(7.05, 3.85, 1.55, 1.05, "FC", r"$1024\to 1024$", "#60a5fa")
    box(8.80, 3.85, 1.15, 1.05, "ReLU", "", "#bfdbfe")
    box(10.15, 3.85, 1.85, 1.05, r"FC + $\tanh$", r"$1024\to 2$", "#93c5fd")
    box(10.35, 2.45, 1.55, 0.95, r"$a=[v,\omega]$", r"$\mathbb{R}^{2}$", "#bbf7d0")
    arr((3.65, 2.95), (3.95, 4.20))
    arr((5.50, 4.37), (5.70, 4.37))
    arr((6.85, 4.37), (7.05, 4.37))
    arr((8.60, 4.37), (8.80, 4.37))
    arr((9.95, 4.37), (10.15, 4.37))
    arr((11.10, 3.85), (11.10, 3.40))

    box(3.95, 1.55, 1.70, 0.95, r"$[s;a]$", r"$152$", "#fde68a")
    arr((2.92, 2.15), (3.95, 2.00))
    arr((10.35, 2.85), (5.65, 2.10))

    box(5.90, 1.55, 2.35, 0.95, r"$Q_{\phi_1}$  MLP", r"$152\to1024\to1024\to 1$", "#fdba74")
    box(5.90, 0.35, 2.35, 0.95, r"$Q_{\phi_2}$  MLP", r"$152\to1024\to1024\to 1$", "#fb923c")
    box(8.55, 0.85, 1.35, 1.15, r"$Q_1$", r"$Q_2$", "#fed7aa")
    box(10.15, 0.75, 1.90, 1.35, r"$\min(Q_1,Q_2)$", r"$y=r+\gamma\min Q'$", "#f5d0fe")
    arr((5.65, 2.00), (5.90, 2.00))
    arr((5.65, 1.85), (5.90, 0.85))
    arr((8.25, 2.00), (8.55, 1.55))
    arr((8.25, 0.82), (8.55, 1.20))
    arr((9.90, 1.40), (10.15, 1.40))

    ax.text(
        6.2,
        5.55,
        r"Actor $\pi_\theta$  (update every 2 critic steps)     ·     batch $B=1024$     ·     hidden $=1024$",
        ha="center",
        fontsize=8.5,
        color="#333",
    )
    return _save(fig, "00_logic_flow.png")


def write_logic_txt():
    txt = textwrap.dedent(
        r"""
        TD3 Record 9 — 拼 PPT 用：每张图单独，按这个顺序接箭头。

        状态怎么排（与 README1 / ros_env 一致）
          s ∈ R^150
          1) L_i,j   i=1..36 水平扇区，j=1..4 仰角层
             先 j 后 i 拉成 144 维（elevation_bin × sector）
          2) ρ, θ, v, ω     （目标距离/方位 + 上一拍线速度角速度）
          3) roll/π, pitch/π

        数据流
          L 矩阵 + (ρθvω) + IMU  →  拼成列向量 s
          s → FC 150→1024 → ReLU → FC 1024→1024 → ReLU → FC 1024→2 → tanh×a_max → a=(v,ω)
          s 与 a 拼接 [s;a] ∈ R^152
          [s;a] → Critic1  152→1024→1024→1  → Q1
          [s;a] → Critic2  152→1024→1024→1  → Q2
          min(Q1,Q2) 做 TD 目标  y = r + γ min Q'

        文件
          00_logic_flow.png     总逻辑（二维方框，无立方体）
          01_L_matrix.png       雷达 4×36 的 L_i,j 方格矩阵（空格子+轴标签）
          01b_L_row.png         单层 j=1：L_1,1 … L_36,1 一排正方形
          02_nav_vector.png     ρ θ v ω 一列正方形
          03_imu_vector.png     roll pitch 一列正方形
          04_state_s_column.png s 怎么叠起来
          05_action_vector.png  v, ω
          06_relu.png           ReLU 曲线
          07_tanh.png           tanh 曲线
          08_h1024_column.png   隐层 1024 维示意（正方形叠，中间省略）
          08b_W_matrix.png      权重矩阵示意（二维方格，不是立方体）
          08c_h1024_grid.png    隐层二维方格
          09_box_concat.png
          10_box_q1.png  11_box_q2.png  12_box_min.png
          13_arrow_right.png  14_arrow_down.png
        """
    ).strip()
    path = os.path.join(OUT, "LOGIC.txt")
    os.makedirs(OUT, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print("wrote", path)


def main():
    write_logic_txt()
    part_logic_sheet()
    part_L_matrix()
    part_L_row()
    part_column([r"$\rho$", r"$\theta$", r"$v$", r"$\omega$"], r"nav  $(\rho,\theta,v,\omega)$", "02_nav_vector.png")
    part_column([r"roll$/\pi$", r"pitch$/\pi$"], "IMU", "03_imu_vector.png", fc="#fce7f3")
    part_s_stack()
    part_column([r"$v$", r"$\omega$"], r"action $a$", "05_action_vector.png", fc="#bbf7d0")
    part_relu()
    part_tanh()
    part_hidden(r"hidden $h\in\mathbb{R}^{1024}$", HIDDEN, "08_h1024_column.png")
    part_weight_matrix()
    part_neuron_grid()
    part_box(r"FC  $150\to 1024$", "Actor layer 1", "09a_fc_150_1024.png", "#93c5fd")
    part_box(r"FC  $1024\to 1024$", "Actor / Critic hidden", "09b_fc_1024_1024.png", "#60a5fa")
    part_box(r"FC  $1024\to 2$", r"Actor out $+\tanh\times a_{\max}$", "09c_fc_1024_2.png", "#93c5fd")
    part_box(r"FC  $152\to 1024$", "Critic layer 1", "09d_fc_152_1024.png", "#fdba74")
    part_box(r"FC  $1024\to 1$", r"Critic out $Q$", "09e_fc_1024_1.png", "#fb923c")
    part_box(r"concat  $[s;a]$", r"$150+2=152$", "09_box_concat.png", "#fde68a")
    part_box(r"$Q_1(s,a)$", r"Critic $\phi_1$  scalar", "10_box_q1.png", "#fdba74")
    part_box(r"$Q_2(s,a)$", r"Critic $\phi_2$  scalar", "11_box_q2.png", "#fb923c")
    part_box(r"$\min(Q_1,Q_2)$", r"$y=r+\gamma\min Q'$", "12_box_min.png", "#f5d0fe", w=3.6)
    part_arrow("13_arrow_right.png", dx=1.5)
    part_arrow("14_arrow_down.png", dx=0.0, dy=-1.2)
    print("OUT dir:", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
