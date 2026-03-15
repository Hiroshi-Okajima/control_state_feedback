"""
pole_placement.py
State Feedback Control: Pole Placement Simulation

Blog article:
    https://blog.control-theory.com/entry/2024/10/01/143612
Hub article:
    https://blog.control-theory.com/entry/state-feedback-control-eng

Repository: control_state_feedback/01_pole_placement/

This script demonstrates:
    1. Step response comparison — closed-loop with different pole placements
    2. Pole map on the s-plane
    3. Effect of pole locations on response speed and oscillation

Plant model (3rd-order controllable canonical form from the blog):
    A = [0 1 0; 0 0 1; -1 1 1],  B = [0; 0; 1],  C = [1 0 0], D = 0
    Open-loop poles: 1 (double), -1   (unstable)
"""

import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import control as ct

# ======================================================================
#  Plot style (self-contained — no external dependency)
# ======================================================================
COLORS = {
    'reference': '#000000', 'method1': '#1f77b4', 'method2': '#d62728',
    'method3':   '#2ca02c', 'method4': '#ff7f0e', 'control': '#9467bd',
    'noise':     '#7f7f7f', 'baseline': '#7f7f7f',
}
FIGSIZE_STANDARD = (8, 5)   # 1200 x 750 px at 150 dpi
FIGSIZE_SQUARE   = (6, 6)   #  900 x 900 px at 150 dpi
DPI = 150

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 13,
    'axes.labelsize': 14, 'legend.fontsize': 12,
    'xtick.labelsize': 12, 'ytick.labelsize': 12,
    'figure.dpi': DPI, 'savefig.dpi': DPI,
    'savefig.facecolor': 'white', 'savefig.edgecolor': 'none',
    'axes.grid': True, 'grid.alpha': 0.3,
})

def save_fig(fig, filepath):
    fig.savefig(filepath, dpi=DPI, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f'Saved: {filepath}')

# ======================================================================
#  Plant definition
# ======================================================================
A = np.array([[0,  1, 0],
              [0,  0, 1],
              [-1, 1, 1]], dtype=float)
B = np.array([[0], [0], [1]], dtype=float)
C = np.array([[1, 0, 0]], dtype=float)
D = np.array([[0]], dtype=float)

n = A.shape[0]

print("Open-loop eigenvalues of A:")
print(np.linalg.eigvals(A))

# ======================================================================
#  Feedback gain designs (pole placement via Ackermann / place)
# ======================================================================
# Design 1: poles at {-1, -2, -3}  (from blog article, K = [5 12 7])
p1 = np.array([-1, -2, -3])
K1 = ct.acker(A, B, p1).reshape(1, -1)
Acl1 = A - B @ K1
print(f"Design 1: K = {K1.flatten()},  CL poles = {np.linalg.eigvals(Acl1).real}")

# Design 2: poles at {-0.5, -1, -2}  (slow — poles near imaginary axis)
p2 = np.array([-0.5, -1, -2])
K2 = ct.acker(A, B, p2).reshape(1, -1)
Acl2 = A - B @ K2

# Design 3: poles at {-5, -6, -7}  (fast — poles far left)
p3 = np.array([-5, -6, -7])
K3 = ct.acker(A, B, p3).reshape(1, -1)
Acl3 = A - B @ K3

# Design 4: poles at {-2, -1+5j, -1-5j}  (oscillatory — large imaginary part)
p4 = np.array([-2, -1+5j, -1-5j])
K4 = ct.acker(A, B, p4).reshape(1, -1).real
Acl4 = A - B @ K4

# ======================================================================
#  Simulation (solve_ivp with RK45)
# ======================================================================
x0 = np.array([1.0, 0.0, 0.0])
Tsim = 8.0


def simulate(Acl_mat, K_mat, x0_vec, Tsim_val):
    """Simulate the closed-loop system dx/dt = (A-BK)x and record u = -Kx."""
    def dynamics(t, x):
        return (Acl_mat @ x.reshape(-1, 1)).flatten()

    sol = solve_ivp(dynamics, [0, Tsim_val], x0_vec,
                    max_step=0.001, dense_output=True)
    t_eval = np.linspace(0, Tsim_val, 8001)
    x_eval = sol.sol(t_eval)  # shape (n, len(t_eval))
    y_eval = (C @ x_eval).flatten()
    u_eval = (-K_mat @ x_eval).flatten()
    return t_eval, x_eval, y_eval, u_eval


t1, X1, Y1, U1 = simulate(Acl1, K1, x0, Tsim)
t2, X2, Y2, U2 = simulate(Acl2, K2, x0, Tsim)
t3, X3, Y3, U3 = simulate(Acl3, K3, x0, Tsim)
t4, X4, Y4, U4 = simulate(Acl4, K4, x0, Tsim)

# ======================================================================
#  Create output directory
# ======================================================================
fig_dir = os.path.join(os.path.dirname(__file__), '..', 'fig')
os.makedirs(fig_dir, exist_ok=True)

# ======================================================================
#  Figure 1: Step response — effect of real-part pole locations
# ======================================================================
fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=FIGSIZE_STANDARD)

ax1a.plot(t1, Y1, color=COLORS['method1'], linewidth=1.5,
          label=f'Design 1: poles = {{{", ".join(map(str, p1.real.astype(int)))}}}')
ax1a.plot(t2, Y2, color=COLORS['method4'], linewidth=1.5,
          label=f'Design 2: poles = {{{", ".join(map(str, p2))}}}')
ax1a.plot(t3, Y3, color=COLORS['method3'], linewidth=1.5,
          label=f'Design 3: poles = {{{", ".join(map(str, p3.real.astype(int)))}}}')
ax1a.axhline(0, color=COLORS['reference'], linewidth=0.8, linestyle='--')
ax1a.set_xlabel('Time [s]')
ax1a.set_ylabel('Output  $y = x_1$')
ax1a.set_title('State Response: Effect of Real-Part Pole Locations')
ax1a.legend(loc='best', framealpha=0.9)
ax1a.grid(True, alpha=0.3)

ax1b.plot(t1, U1, color=COLORS['method1'], linewidth=1.5, label='Design 1')
ax1b.plot(t2, U2, color=COLORS['method4'], linewidth=1.5, label='Design 2')
ax1b.plot(t3, U3, color=COLORS['method3'], linewidth=1.5, label='Design 3')
ax1b.set_xlabel('Time [s]')
ax1b.set_ylabel('Control input  $u$')
ax1b.set_title('Control Input')
ax1b.legend(loc='best', framealpha=0.9)
ax1b.grid(True, alpha=0.3)

plt.tight_layout()
save_fig(fig1, os.path.join(fig_dir, 'pole_placement_step_response.png'))

# ======================================================================
#  Figure 2: Pole map on s-plane
# ======================================================================
fig2, ax2 = plt.subplots(1, 1, figsize=FIGSIZE_SQUARE)

ol_poles = np.linalg.eigvals(A)
cl1_poles = np.linalg.eigvals(Acl1)
cl2_poles = np.linalg.eigvals(Acl2)
cl3_poles = np.linalg.eigvals(Acl3)
cl4_poles = np.linalg.eigvals(Acl4)

ax2.plot(ol_poles.real, ol_poles.imag, 'kx', markersize=12, markeredgewidth=2.0,
         label='Open-loop')
ax2.plot(cl1_poles.real, cl1_poles.imag, 'o', color=COLORS['method1'],
         markersize=10, markeredgewidth=1.5, markerfacecolor='none',
         label='Design 1: {-1, -2, -3}')
ax2.plot(cl2_poles.real, cl2_poles.imag, 's', color=COLORS['method4'],
         markersize=10, markeredgewidth=1.5, markerfacecolor='none',
         label='Design 2: {-0.5, -1, -2}')
ax2.plot(cl3_poles.real, cl3_poles.imag, 'D', color=COLORS['method3'],
         markersize=10, markeredgewidth=1.5, markerfacecolor='none',
         label='Design 3: {-5, -6, -7}')
ax2.plot(cl4_poles.real, cl4_poles.imag, '^', color=COLORS['method2'],
         markersize=10, markeredgewidth=1.5, markerfacecolor='none',
         label=r'Design 4: {-2, -1$\pm$5j}')

ax2.axvline(0, color='k', linewidth=0.8, linestyle='--')
ax2.axhline(0, color='k', linewidth=0.8, linestyle='--')
ax2.set_xlabel('Real')
ax2.set_ylabel('Imaginary')
ax2.set_title('Pole Map (s-plane)')
ax2.legend(loc='best', framealpha=0.9)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
save_fig(fig2, os.path.join(fig_dir, 'pole_placement_pole_map.png'))

# ======================================================================
#  Figure 3: Effect of imaginary part (Design 1 vs Design 4)
# ======================================================================
fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=FIGSIZE_STANDARD)

ax3a.plot(t1, Y1, color=COLORS['method1'], linewidth=1.5,
          label='Design 1: poles = {-1, -2, -3}')
ax3a.plot(t4, Y4, color=COLORS['method2'], linewidth=1.5,
          label=r'Design 4: poles = {-2, -1$\pm$5j}')
ax3a.axhline(0, color=COLORS['reference'], linewidth=0.8, linestyle='--')
ax3a.set_xlabel('Time [s]')
ax3a.set_ylabel('Output  $y = x_1$')
ax3a.set_title('Effect of Imaginary-Part Pole Locations')
ax3a.legend(loc='best', framealpha=0.9)
ax3a.grid(True, alpha=0.3)

ax3b.plot(t1, U1, color=COLORS['method1'], linewidth=1.5, label='Design 1')
ax3b.plot(t4, U4, color=COLORS['method2'], linewidth=1.5, label='Design 4')
ax3b.set_xlabel('Time [s]')
ax3b.set_ylabel('Control input  $u$')
ax3b.set_title('Control Input')
ax3b.legend(loc='best', framealpha=0.9)
ax3b.grid(True, alpha=0.3)

plt.tight_layout()
save_fig(fig3, os.path.join(fig_dir, 'pole_placement_imaginary_effect.png'))

print("\nAll figures saved to fig/ folder.")
plt.show()
