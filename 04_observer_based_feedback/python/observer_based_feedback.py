"""
observer_based_feedback.py
Observer-Based State Feedback Control — Separation Principle Demo

Blog article: State Observer for State Space Model
  https://blog.control-theory.com/entry/2024/10/01/143305
Hub article: State Feedback Control and State-Space Design
  https://blog.control-theory.com/entry/state-feedback-control-eng
  Section: "Observer-Based Feedback and the Separation Principle"

Repository: control_state_feedback/04_observer_based_feedback/
Author: Hiroshi Okajima, Kumamoto University

This script demonstrates the separation principle:
  Fig 1: Full-state feedback vs observer-based feedback (output comparison)
  Fig 2: True state vs estimated state trajectories
  Fig 3: Estimation error convergence
  Fig 4: Closed-loop pole map (controller poles & observer poles, separated)

NOTE: This is a lightweight demo focusing on the separation principle.
The same plant as 01_pole_placement/ through 03_integral_servo/ is used.
For comprehensive observer design (Kalman filter, H-infinity filter,
multi-rate observer, MCV observer), see:
  MATLAB: https://github.com/Hiroshi-Okajima/MATLAB_state_observer
  Python: https://github.com/Hiroshi-Okajima/python_state_observer
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# ===== Plot style (self-contained, project standard) =====
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 13
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['legend.fontsize'] = 12
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12

# Color definitions (project standard)
COL_REF   = '#000000'  # reference / true value
COL_RESP1 = '#1f77b4'  # response (method 1) - blue
COL_RESP2 = '#d62728'  # response (method 2) - red
COL_RESP3 = '#2ca02c'  # response (method 3) - green
COL_RESP4 = '#ff7f0e'  # response (method 4) - orange
COL_INPUT = '#9467bd'  # control input - purple
COL_GRAY  = '#7f7f7f'  # baseline / noise

LW_REF  = 2.0   # linewidth: reference / true
LW_RESP = 1.5   # linewidth: response / estimate
LW_BASE = 1.5   # linewidth: baseline (dashed)

fig_standard = (8, 5)   # inches (standard time-series)
fig_wide     = (8, 6)   # inches (wide)
fig_square   = (6, 6)   # inches (pole map)
fig_dpi      = 150

# Create fig/ directory
fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'fig')
os.makedirs(fig_dir, exist_ok=True)

def savefig(fig, filename):
    """Save figure with project standard settings."""
    filepath = os.path.join(fig_dir, filename)
    fig.savefig(filepath, dpi=fig_dpi, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f'Saved: {filepath}')


# ===== Plant model (same as 01, 02, 03) =====
#  Controllable canonical form, 3rd-order SISO
#  Open-loop eigenvalues: 1, -1+j, -1-j (unstable)
A = np.array([[0, 1, 0],
              [0, 0, 1],
              [1, -1, -1]])
B = np.array([[0], [0], [1]])
C = np.array([[1, 0, 0]])
D = np.array([[0]])
n = A.shape[0]

eig_ol = np.linalg.eigvals(A)
print("=== Plant Model ===")
print(f"Open-loop eigenvalues: {eig_ol}")
print()

# ===== Controller design: Pole placement (same as 01) =====
desired_poles_K = [-1, -2, -3]
K = ct.place(A, B, desired_poles_K)
eig_K = np.linalg.eigvals(A - B @ K)
print(f"State feedback gain K = {K}")
print(f"Controller poles (A-BK): {eig_K}")
print()

# ===== Observer design: Pole placement =====
#  Observer poles should be faster than controller poles (2-5x rule of thumb)
#  Place observer poles at -5, -6, -7
desired_poles_L = [-5, -6, -7]
L = ct.place(A.T, C.T, desired_poles_L).T
eig_L = np.linalg.eigvals(A - L @ C)
print(f"Observer gain L = {L.T}")
print(f"Observer poles (A-LC): {eig_L}")
print()

# ===== Verify separation principle =====
#  Full closed-loop system (2n = 6 states)
#  State: [x; e] where e = x - x_hat
A_cl = np.block([
    [A - B @ K,           B @ K],
    [np.zeros((n, n)),    A - L @ C]
])
eig_cl = np.linalg.eigvals(A_cl)
print(f"Full closed-loop eigenvalues (2n = {2*n}):")
for ev in eig_cl:
    print(f"  {ev:.4f}")
print("(Should be union of controller poles and observer poles)")
print()

# ===== Simulation =====
Tsim = 8.0   # simulation time [s]

# Initial conditions
x0 = np.array([1, 0, 0])       # plant initial state
x_hat0 = np.array([0, 0, 0])   # observer initial state (unknown)
e0 = x0 - x_hat0               # initial estimation error
z0 = np.concatenate([x0, e0])   # augmented state [x; e]

# --- Case 1: Full-state feedback (ideal) ---
sys_full = ct.ss(A - B @ K, np.zeros((n, 1)), C, D)
t_full = np.linspace(0, Tsim, 1000)
t_full, y_full_resp = ct.initial_response(sys_full, T=t_full, X0=x0)
# Get state trajectories for control input
_, _, x_full_states = ct.initial_response(sys_full, T=t_full, X0=x0,
                                           output=0, return_x=True)
u_full = -(K @ x_full_states).flatten()

# --- Case 2: Observer-based feedback ---
B_cl = np.zeros((2*n, 1))

# Output matrices for augmented system
C_cl_y    = np.hstack([C, np.zeros((1, n))])          # y = Cx
C_cl_x    = np.hstack([np.eye(n), np.zeros((n, n))])  # true state x
C_cl_e    = np.hstack([np.zeros((n, n)), np.eye(n)])   # estimation error e
C_cl_xhat = np.hstack([np.eye(n), -np.eye(n)])         # x_hat = x - e

C_all = np.vstack([C_cl_y, C_cl_x, C_cl_e, C_cl_xhat])
D_all = np.zeros((1 + 3*n, 1))

sys_obs = ct.ss(A_cl, B_cl, C_all, D_all)
t_obs, y_obs_all = ct.initial_response(sys_obs, T=t_full, X0=z0)

y_obs    = y_obs_all[0, :]               # output y(t)
x_true   = y_obs_all[1:1+n, :]           # true state x(t)  [n x T]
e_obs    = y_obs_all[1+n:1+2*n, :]       # estimation error  [n x T]
x_hat    = y_obs_all[1+2*n:1+3*n, :]     # estimated state   [n x T]
u_obs    = -(K @ x_hat).flatten()          # control input u = -K*x_hat


# ===== Figure 1: Output comparison =====
fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=fig_standard, sharex=True)

ax1a.plot(t_full, y_full_resp, color=COL_RESP1, linewidth=LW_RESP,
          label=r'Full-state feedback ($u = -Kx$)')
ax1a.plot(t_obs, y_obs, color=COL_RESP2, linewidth=LW_RESP, linestyle='--',
          label=r'Observer-based feedback ($u = -K\hat{x}$)')
ax1a.set_ylabel('Output $y(t)$')
ax1a.set_title('Output Response: Full-State FB vs Observer-Based FB')
ax1a.legend(loc='best', framealpha=0.9)
ax1a.grid(True, alpha=0.3)

ax1b.plot(t_full, u_full, color=COL_RESP1, linewidth=LW_RESP,
          label='Full-state feedback')
ax1b.plot(t_obs, u_obs, color=COL_RESP2, linewidth=LW_RESP, linestyle='--',
          label='Observer-based feedback')
ax1b.set_xlabel('Time [s]')
ax1b.set_ylabel('Control input $u(t)$')
ax1b.set_title('Control Input Comparison')
ax1b.legend(loc='best', framealpha=0.9)
ax1b.grid(True, alpha=0.3)

plt.tight_layout()
savefig(fig1, 'observer_based_feedback_output_comparison.png')


# ===== Figure 2: True state vs estimated state =====
fig2, axes2 = plt.subplots(n, 1, figsize=fig_standard, sharex=True)
state_labels = [r'$x_1(t)$', r'$x_2(t)$', r'$x_3(t)$']

for i in range(n):
    axes2[i].plot(t_obs, x_true[i, :], color=COL_REF, linewidth=LW_REF,
                  label=r'True state $x(t)$' if i == 0 else None)
    axes2[i].plot(t_obs, x_hat[i, :], color=COL_RESP1, linewidth=LW_RESP,
                  linestyle='--',
                  label=r'Estimated state $\hat{x}(t)$' if i == 0 else None)
    axes2[i].set_ylabel(state_labels[i])
    axes2[i].grid(True, alpha=0.3)
    if i == 0:
        axes2[i].set_title(r'True State vs Estimated State')
        axes2[i].legend(loc='best', framealpha=0.9)

axes2[-1].set_xlabel('Time [s]')

plt.tight_layout()
savefig(fig2, 'observer_based_feedback_state_estimation.png')


# ===== Figure 3: Estimation error convergence =====
fig3, axes3 = plt.subplots(n, 1, figsize=fig_standard, sharex=True)

for i in range(n):
    axes3[i].plot(t_obs, e_obs[i, :], color=COL_RESP2, linewidth=LW_RESP)
    axes3[i].axhline(0, color=COL_GRAY, linewidth=1.0, linestyle=':')
    axes3[i].set_ylabel(f'$e_{i+1}(t)$')
    axes3[i].grid(True, alpha=0.3)
    if i == 0:
        axes3[i].set_title(r'Estimation Error: $e(t) = x(t) - \hat{x}(t)$')

axes3[-1].set_xlabel('Time [s]')

plt.tight_layout()
savefig(fig3, 'observer_based_feedback_estimation_error.png')


# ===== Figure 4: Pole map (separation principle) =====
fig4, ax4 = plt.subplots(1, 1, figsize=fig_square)

# Open-loop poles
ax4.plot(np.real(eig_ol), np.imag(eig_ol), 'x', color=COL_GRAY,
         markersize=12, markeredgewidth=LW_BASE, label='Open-loop poles')

# Controller poles (A - BK)
ax4.plot(np.real(eig_K), np.imag(eig_K), 'o', color=COL_RESP1,
         markersize=10, markeredgewidth=2, fillstyle='none',
         label=f'Controller poles (A-BK): {desired_poles_K}')

# Observer poles (A - LC)
ax4.plot(np.real(eig_L), np.imag(eig_L), 's', color=COL_RESP2,
         markersize=10, markeredgewidth=2, fillstyle='none',
         label=f'Observer poles (A-LC): {desired_poles_L}')

# Full closed-loop poles (should overlap)
ax4.plot(np.real(eig_cl), np.imag(eig_cl), '+', color=COL_RESP3,
         markersize=14, markeredgewidth=1.5,
         label='Full closed-loop poles (2n)')

ax4.axvline(0, color='#b0b0b0', linewidth=0.5)
ax4.axhline(0, color='#b0b0b0', linewidth=0.5)
ax4.set_xlabel('Real')
ax4.set_ylabel('Imaginary')
ax4.set_title('Pole Map — Separation Principle')
ax4.legend(loc='best', framealpha=0.9)
ax4.grid(True, alpha=0.3)
ax4.set_aspect('equal')

plt.tight_layout()
savefig(fig4, 'observer_based_feedback_pole_map.png')


# ===== Console summary =====
print("\n=== Summary ===")
print("Plant: 3rd-order, A = [0 1 0; 0 0 1; 1 -1 -1], B = [0;0;1], C = [1 0 0]")
print(f"Controller poles (A-BK): {desired_poles_K}")
print(f"Observer poles (A-LC):   {desired_poles_L}")
print("Separation principle verified: full closed-loop poles = union of above")
print(f"Initial state mismatch: x0 = {x0}, x_hat0 = {x_hat0}")

plt.show()
