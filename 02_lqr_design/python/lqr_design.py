
"""
lqr_design.py
State Feedback Control: LQR (Optimal Regulator) Design

Blog article:
    https://blog.control-theory.com/entry/2024/10/01/143503
Hub article:
    https://blog.control-theory.com/entry/state-feedback-control-eng

Repository: control_state_feedback/02_lqr_design/

This script demonstrates:
    1. LQR response comparison with different R weights (Q = I fixed)
    2. State and control input trade-off visualization
    3. Pole map comparison (LQR vs pole placement)
    4. Cost function decomposition vs R weight

Plant model (same 3rd-order system as 01_pole_placement):
    A = [0 1 0; 0 0 1; -1 1 1],  B = [0; 0; 1],  C = [1 0 0], D = 0
    Open-loop poles: 1 (double), -1   (unstable)
"""

import os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
import control as ct

# ======================================================================
#  Plot style (self-contained)
# ======================================================================
COLORS = {
    'reference': '#000000', 'method1': '#1f77b4', 'method2': '#d62728',
    'method3':   '#2ca02c', 'method4': '#ff7f0e', 'control': '#9467bd',
    'noise':     '#7f7f7f', 'baseline': '#7f7f7f',
}
FIGSIZE_STANDARD = (8, 5)
FIGSIZE_WIDE     = (8, 6)
FIGSIZE_SQUARE   = (6, 6)
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
#  Create output directory
# ======================================================================
fig_dir = os.path.join(os.path.dirname(__file__), '..', 'fig')
os.makedirs(fig_dir, exist_ok=True)

# ======================================================================
#  Plant definition (same as 01_pole_placement)
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
#  LQR designs with different R weights  (Q = I fixed)
# ======================================================================
Q = np.eye(n)

R_vals   = [0.01, 1.0, 100.0]
R_labels = ['R = 0.01 (fast)', 'R = 1 (balanced)', 'R = 100 (energy-saving)']
R_colors = [COLORS['method1'], COLORS['method2'], COLORS['method3']]

K_list = []
P_list = []
for Rv in R_vals:
    R_mat = np.array([[Rv]])
    K_lqr, _, _ = ct.lqr(A, B, Q, R_mat)
    P_riccati = solve_continuous_are(A, B, Q, R_mat)
    K_list.append(K_lqr)
    P_list.append(P_riccati)
    Acl = A - B @ K_lqr
    print(f"R = {Rv}:  K = {K_lqr.flatten()},  "
          f"CL poles = {np.sort(np.linalg.eigvals(Acl).real)}")

# ======================================================================
#  Simulation
# ======================================================================
x0 = np.array([1.0, 0.0, 0.0])
Tsim = 8.0


def simulate(Acl_mat, K_mat, x0_vec, Tsim_val):
    def dynamics(t, x):
        return (Acl_mat @ x.reshape(-1, 1)).flatten()
    sol = solve_ivp(dynamics, [0, Tsim_val], x0_vec,
                    max_step=0.001, dense_output=True)
    t_eval = np.linspace(0, Tsim_val, 8001)
    x_eval = sol.sol(t_eval)
    y_eval = (C @ x_eval).flatten()
    u_eval = (-K_mat @ x_eval).flatten()
    return t_eval, x_eval, y_eval, u_eval


results = []
for Ki in K_list:
    Acl_i = A - B @ Ki
    results.append(simulate(Acl_i, Ki, x0, Tsim))

# ======================================================================
#  Figure 1: Output response comparison (different R)
# ======================================================================
fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=FIGSIZE_STANDARD)

for i, (ti, Xi, Yi, Ui) in enumerate(results):
    ax1a.plot(ti, Yi, color=R_colors[i], linewidth=1.5, label=R_labels[i])
ax1a.axhline(0, color=COLORS['reference'], linewidth=0.8, linestyle='--')
ax1a.set_xlabel('Time [s]')
ax1a.set_ylabel('Output  $y = x_1$')
ax1a.set_title('LQR Response: Effect of R Weight  ($Q = I$)')
ax1a.legend(loc='best', framealpha=0.9)

for i, (ti, Xi, Yi, Ui) in enumerate(results):
    ax1b.plot(ti, Ui, color=R_colors[i], linewidth=1.5, label=R_labels[i])
ax1b.set_xlabel('Time [s]')
ax1b.set_ylabel('Control input  $u$')
ax1b.set_title('Control Input')
ax1b.legend(loc='best', framealpha=0.9)

plt.tight_layout()
save_fig(fig1, os.path.join(fig_dir, 'lqr_design_R_comparison.png'))

# ======================================================================
#  Figure 2: All three state variables  (R=0.01 vs R=100)
# ======================================================================
fig2, axes2 = plt.subplots(3, 1, figsize=FIGSIZE_WIDE)
state_labels = [r'$x_1$', r'$x_2$', r'$x_3$']

for j in range(3):
    axes2[j].plot(results[0][0], results[0][1][j, :],
                  color=COLORS['method1'], linewidth=1.5, label='R = 0.01')
    axes2[j].plot(results[2][0], results[2][1][j, :],
                  color=COLORS['method3'], linewidth=1.5, label='R = 100')
    axes2[j].axhline(0, color=COLORS['reference'], linewidth=0.8, linestyle='--')
    axes2[j].set_ylabel(state_labels[j])
    axes2[j].grid(True, alpha=0.3)
    if j == 0:
        axes2[j].set_title('State Variables: R = 0.01 vs R = 100')
        axes2[j].legend(loc='best', framealpha=0.9)
    if j == 2:
        axes2[j].set_xlabel('Time [s]')

plt.tight_layout()
save_fig(fig2, os.path.join(fig_dir, 'lqr_design_state_variables.png'))

# ======================================================================
#  Figure 3: Pole map comparison  (LQR vs pole placement from 01)
# ======================================================================
fig3, ax3 = plt.subplots(1, 1, figsize=FIGSIZE_SQUARE)

ol_poles = np.linalg.eigvals(A)
ax3.plot(ol_poles.real, ol_poles.imag, 'kx', markersize=12,
         markeredgewidth=2.0, label='Open-loop')

markers = ['o', 's', 'D']
for i, Ki in enumerate(K_list):
    Acl_i = A - B @ Ki
    cl_poles = np.linalg.eigvals(Acl_i)
    ax3.plot(cl_poles.real, cl_poles.imag, markers[i], color=R_colors[i],
             markersize=10, markeredgewidth=1.5, markerfacecolor='none',
             label=R_labels[i])

# Pole placement Design 1 from 01 for reference
K_pp = ct.acker(A, B, [-1, -2, -3]).reshape(1, -1)
Acl_pp = A - B @ K_pp
cl_pp = np.linalg.eigvals(Acl_pp)
ax3.plot(cl_pp.real, cl_pp.imag, '^', color=COLORS['method4'],
         markersize=10, markeredgewidth=1.5, markerfacecolor='none',
         label='Pole place {-1,-2,-3}')

ax3.axvline(0, color='k', linewidth=0.8, linestyle='--')
ax3.axhline(0, color='k', linewidth=0.8, linestyle='--')
ax3.set_xlabel('Real')
ax3.set_ylabel('Imaginary')
ax3.set_title('Pole Map: LQR Designs vs Pole Placement')
ax3.legend(loc='best', framealpha=0.9)
ax3.set_aspect('equal')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
save_fig(fig3, os.path.join(fig_dir, 'lqr_design_pole_map.png'))

# ======================================================================
#  Figure 4: Cost function trade-off  (J vs R)
# ======================================================================
R_sweep = np.logspace(-3, 3, 50)
J_state = np.zeros_like(R_sweep)
J_input = np.zeros_like(R_sweep)

dt_sim = Tsim / 8000
for idx, Rv in enumerate(R_sweep):
    R_mat = np.array([[Rv]])
    Ki_sw, _, _ = ct.lqr(A, B, Q, R_mat)
    Acl_sw = A - B @ Ki_sw
    _, Xi_sw, _, Ui_sw = simulate(Acl_sw, Ki_sw, x0, Tsim)
    J_state[idx] = dt_sim * np.sum(np.sum((Q @ Xi_sw) * Xi_sw, axis=0))
    J_input[idx] = dt_sim * Rv * np.sum(Ui_sw ** 2)

fig4, (ax4a, ax4b) = plt.subplots(2, 1, figsize=FIGSIZE_STANDARD)

ax4a.semilogx(R_sweep, J_state, color=COLORS['method1'], linewidth=1.5,
              label=r'$\int x^T Q x \, dt$')
ax4a.semilogx(R_sweep, J_input, color=COLORS['method2'], linewidth=1.5,
              label=r'$\int u^T R u \, dt$')
ax4a.semilogx(R_sweep, J_state + J_input, color=COLORS['reference'],
              linewidth=2.0, linestyle='--', label='Total $J$')
ax4a.set_xlabel('Weight $R$')
ax4a.set_ylabel('Cost')
ax4a.set_title('Cost Function Decomposition vs R Weight')
ax4a.legend(loc='best', framealpha=0.9)

ax4b.semilogx(R_sweep, J_state, color=COLORS['method1'], linewidth=1.5,
              label=r'$\int x^T Q x \, dt$  (state penalty)')
ax4b.semilogx(R_sweep, J_input / R_sweep, color=COLORS['control'], linewidth=1.5,
              label=r'$\int u^2 \, dt$  (input energy)')
ax4b.set_xlabel('Weight $R$')
ax4b.set_ylabel('Value')
ax4b.set_title(r'State Cost vs Input Energy ($\int u^2 dt$)')
ax4b.legend(loc='best', framealpha=0.9)

plt.tight_layout()
save_fig(fig4, os.path.join(fig_dir, 'lqr_design_cost_tradeoff.png'))

print("\nAll figures saved to fig/ folder.")
plt.show()
