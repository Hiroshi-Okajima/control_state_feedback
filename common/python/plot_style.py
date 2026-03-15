
"""
Common plot style for control_state_feedback repository.

Usage:
    import sys; sys.path.append('../common/python')
    from plot_style import apply_style, COLORS, save_fig

Reference: Project instruction document (plot style specification).
"""

import matplotlib.pyplot as plt
import matplotlib

# ---------------------------------------------------------------------------
# Color scheme (unified across all clusters)
# ---------------------------------------------------------------------------
COLORS = {
    'reference':   '#000000',   # True value / reference (black)
    'method1':     '#1f77b4',   # Response (method 1) — blue
    'method2':     '#d62728',   # Response (method 2) — red
    'method3':     '#2ca02c',   # Response (method 3) — green
    'method4':     '#ff7f0e',   # Response (method 4) — orange
    'control':     '#9467bd',   # Control input — purple
    'noise':       '#7f7f7f',   # Noise / disturbance — gray
    'baseline':    '#7f7f7f',   # No compensation — gray
}

# ---------------------------------------------------------------------------
# Figure sizes (inches) — optimised for Hatena Blog (~700 px body width)
# ---------------------------------------------------------------------------
FIGSIZE_STANDARD = (8, 5)    # 1200 x 750 px at 150 dpi
FIGSIZE_WIDE     = (8, 6)    # 1200 x 900 px (Bode plots, etc.)
FIGSIZE_SQUARE   = (6, 6)    #  900 x 900 px (pole maps, etc.)

DPI = 150


def apply_style():
    """Apply the unified plot style to matplotlib."""
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 13
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['legend.fontsize'] = 12
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 12
    plt.rcParams['figure.dpi'] = DPI
    plt.rcParams['savefig.dpi'] = DPI
    plt.rcParams['savefig.facecolor'] = 'white'
    plt.rcParams['savefig.edgecolor'] = 'none'
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3


def save_fig(fig, filepath):
    """Save figure with the project-standard settings."""
    fig.savefig(filepath, dpi=DPI, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f'Saved: {filepath}')
