# -*- coding: utf-8 -*-
"""Shared matplotlib paper style for Stage-A TD3 research figures.

English labels + Times New Roman (fallback: Liberation Serif / Nimbus Roman).
"""

from __future__ import print_function


def apply_paper_style():
    """Set global rcParams for publication-style English figures."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = [
        "Times New Roman",
        "Liberation Serif",
        "Nimbus Roman",
        "Nimbus Roman No9 L",
        "DejaVu Serif",
    ]
    plt.rcParams["mathtext.fontset"] = "stix"
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["ps.fonttype"] = 42
    return True
