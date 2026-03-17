"""
integral_servo.py
Integral-Type Servo System for State Feedback Control

Blog article (Hub): State Feedback Control and State-Space Design
  https://blog.control-theory.com/entry/state-feedback-control-eng
  Section: "Integral-Type Servo System"

Repository: control_state_feedback/03_integral_servo/
Author: Hiroshi Okajima, Kumamoto University

This script demonstrates:
  Fig 1: Step response — regulator (no integrator) vs integral servo
  Fig 2: Effect of integral weight on transient response (3 designs)
  Fig 3: Tracking error convergence and integrator state
  Fig 4: Pole map comparing all designs

Plant model: Same 3rd-order system as 01_pole_placement/ and 02_lqr_design/
  A = [0 1 0; 0 0 1; 1 -1 -1],  B = [0; 0; 1],  C = [1 0 0]
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct
from scipy.linalg import solve_continuous_are

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


# ===== Plant model (same as 01, 02) =====
#  Controllable canonical form, 3rd-order SISO
A = np.array([[0, 1, 0],
              [0, 0, 1],
              [1, -1, -1]])
B = np.array([[0], [0], [1]])
C = np.array([[1, 0, 0]])
D = np.array([[0]])
n = A.shape[0]       # state dimension = 3
m = B.shape[1]       # input dimension = 1
l = C.shape[0]       # output dimension = 1

eig_ol = np.linalg.eigvals(A)
print("=== Plant Model ===")
print(f"Open-loop eigenvalues: {eig_ol}")
print()

# ===== Case 1: State feedback regulator (NO integral action) =====
# u = -K_reg*x + N_bar*r  (feedforward for reference tracking attempt)
desired_poles_reg = [-1, -2, -3]
K_reg = ct.place(A, B, desired_poles_reg)
A_cl_reg = A - B @ K_reg
# Feedforward gain for steady-state matching:
#   y_ss = -C*(A-BK)^{-1}*B * v = r  =>  N_bar = -1/(C*(A-BK)^{-1}*B)
N_bar = -1.0 / (C @ np.linalg.solve(A_cl_reg, B))[0, 0]
print(f"Regulator gain K_reg = {K_reg}")
print(f"Feedforward gain N_bar = {N_bar:.4f}")
print("(N_bar provides exact tracking only for nominal plant)")
print()

# ===== Augmented system for integral-type servo =====
#  Augmented state: x_a = [x; x_I], dot(x_I) = r - y = r - C*x
#  A_a = [A, 0; -C, 0],  B_a = [B; 0]
A_a = np.block([[A, np.zeros((n, l))],
                [-C, np.zeros((l, l))]])
B_a = np.block([[B],
                [np.zeros((l, m))]])
n_a = n + l  # augmented dimension = 4

print("=== Augmented System (Integral Servo) ===")
print(f"Dimension: n_a = {n_a} (original {n} + integrator {l})")

# Check controllability
Uc_a = ct.ctrb(A_a, B_a)
rank_Uc = np.linalg.matrix_rank(Uc_a)
print(f"Controllability rank of (A_a, B_a): {rank_Uc} (need {n_a})")
print()

# ===== Design 1: Integral servo via pole placement =====
desired_poles_servo = [-1, -2, -3, -1.5]
K_a1 = ct.place(A_a, B_a, desired_poles_servo)
K_x1 = K_a1[0, :n]      # state feedback part
K_I1 = K_a1[0, n:]       # integral gain part

eig_s1 = np.linalg.eigvals(A_a - B_a @ K_a1)
print("--- Design 1: Pole Placement ---")
print(f"Desired poles: {desired_poles_servo}")
print(f"K_x = {K_x1},  K_I = {K_I1}")
print(f"Closed-loop poles: {eig_s1}")
print()

# ===== Design 2: Integral servo via LQR (moderate integral weight) =====
Q2 = np.diag([1, 1, 1, 10])
R2 = np.array([[1]])
K_a2, _, _ = ct.lqr(A_a, B_a, Q2, R2)
K_x2 = K_a2[0, :n]
K_I2 = K_a2[0, n:]

eig_s2 = np.linalg.eigvals(A_a - B_a @ K_a2)
print("--- Design 2: LQR (Q_I = 10) ---")
print(f"K_x = {K_x2},  K_I = {K_I2}")
print(f"Closed-loop poles: {eig_s2}")
print()

# ===== Design 3: Integral servo via LQR (strong integral weight) =====
Q3 = np.diag([1, 1, 1, 100])
R3 = np.array([[1]])
K_a3, _, _ = ct.lqr(A_a, B_a, Q3, R3)
K_x3 = K_a3[0, :n]
K_I3 = K_a3[0, n:]

eig_s3 = np.linalg.eigvals(A_a - B_a @ K_a3)
print("--- Design 3: LQR (Q_I = 100) ---")
print(f"K_x = {K_x3},  K_I = {K_I3}")
print(f"Closed-loop poles: {eig_s3}")
print()

# ===== Simulation =====
Tsim = 10.0    # simulation time [s]
r_val = 1.0    # step reference value
dt = 0.01
t = np.arange(0, Tsim + dt, dt)
N = len(t)
r = r_val * np.ones(N)

# --- Case 1: Regulator with feedforward (no integrator) ---
x_reg = np.zeros((n, N))
u_reg = np.zeros(N)
y_reg = np.zeros(N)

for k in range(N - 1):
    y_reg[k] = (C @ x_reg[:, k])[0]
    u_reg[k] = (-K_reg @ x_reg[:, k] + N_bar * r[k])[0]
    dx = A @ x_reg[:, k] + B[:, 0] * u_reg[k]
    x_reg[:, k + 1] = x_reg[:, k] + dt * dx
y_reg[-1] = (C @ x_reg[:, -1])[0]
u_reg[-1] = (-K_reg @ x_reg[:, -1] + N_bar * r[-1])[0]


def simulate_integral_servo(K_a_gain):
    """Simulate integral-type servo system with given augmented gain."""
    x_a = np.zeros((n_a, N))
    u = np.zeros(N)
    y = np.zeros(N)

    for k in range(N - 1):
        y[k] = (C @ x_a[:n, k])[0]
        u[k] = (-K_a_gain @ x_a[:, k])[0]
        dx_plant = A @ x_a[:n, k] + B[:, 0] * u[k]
        dx_integ = r[k] - y[k]
        x_a[:, k + 1] = x_a[:, k] + dt * np.concatenate([dx_plant, [dx_integ]])
    y[-1] = (C @ x_a[:n, -1])[0]
    u[-1] = (-K_a_gain @ x_a[:, -1])[0]

    return y, u, x_a


y_s1, u_s1, x_a1_sim = simulate_integral_servo(K_a1)
y_s2, u_s2, x_a2_sim = simulate_integral_servo(K_a2)
y_s3, u_s3, x_a3_sim = simulate_integral_servo(K_a3)

# --- Additional: Simulate with step disturbance at t=5 ---
#  Plant: dx/dt = Ax + Bu + B*d,  d = 0.5 for t >= 5
d_val = 0.5
t_dist = 5.0

# Regulator with feedforward + disturbance
x_reg_d = np.zeros((n, N))
u_reg_d = np.zeros(N)
y_reg_d = np.zeros(N)
for k in range(N - 1):
    y_reg_d[k] = (C @ x_reg_d[:, k])[0]
    u_reg_d[k] = (-K_reg @ x_reg_d[:, k] + N_bar * r[k])[0]
    d_k = d_val if t[k] >= t_dist else 0.0
    dx = A @ x_reg_d[:, k] + B[:, 0] * (u_reg_d[k] + d_k)
    x_reg_d[:, k + 1] = x_reg_d[:, k] + dt * dx
y_reg_d[-1] = (C @ x_reg_d[:, -1])[0]
u_reg_d[-1] = (-K_reg @ x_reg_d[:, -1] + N_bar * r[-1])[0]


def simulate_integral_servo_dist(K_a_gain):
    """Simulate integral-type servo with step disturbance."""
    x_a = np.zeros((n_a, N))
    u = np.zeros(N)
    y = np.zeros(N)
    for k in range(N - 1):
        y[k] = (C @ x_a[:n, k])[0]
        u[k] = (-K_a_gain @ x_a[:, k])[0]
        d_k = d_val if t[k] >= t_dist else 0.0
        dx_plant = A @ x_a[:n, k] + B[:, 0] * (u[k] + d_k)
        dx_integ = r[k] - y[k]
        x_a[:, k + 1] = x_a[:, k] + dt * np.concatenate([dx_plant, [dx_integ]])
    y[-1] = (C @ x_a[:n, -1])[0]
    u[-1] = (-K_a_gain @ x_a[:, -1])[0]
    return y, u, x_a


y_s2_d, u_s2_d, _ = simulate_integral_servo_dist(K_a2)

# ===== Figure 1: Step response comparison (with/without integrator) =====
fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=fig_standard)

ax1a.plot(t, r, color=COL_REF, linewidth=LW_REF, label='Reference r(t)')
ax1a.plot(t, y_reg, color=COL_GRAY, linewidth=LW_BASE, linestyle='--',
          label='Regulator + feedforward (no integrator)')
ax1a.plot(t, y_s1, color=COL_RESP2, linewidth=LW_RESP,
          label='Integral servo (pole placement)')
ax1a.set_xlabel('Time [s]')
ax1a.set_ylabel('Output y(t)')
ax1a.set_title('Step Response: Regulator vs Integral Servo')
ax1a.legend(loc='best', framealpha=0.9)
ax1a.grid(True, alpha=0.3)

ax1b.plot(t, u_reg, color=COL_GRAY, linewidth=LW_BASE, linestyle='--',
          label='Regulator + feedforward')
ax1b.plot(t, u_s1, color=COL_RESP2, linewidth=LW_RESP,
          label='Integral servo')
ax1b.set_xlabel('Time [s]')
ax1b.set_ylabel('Control input u(t)')
ax1b.set_title('Control Input')
ax1b.legend(loc='best', framealpha=0.9)
ax1b.grid(True, alpha=0.3)

plt.tight_layout()
savefig(fig1, 'integral_servo_step_comparison.png')

# ===== Figure 2: Integral servo with different designs =====
fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=fig_standard)

ax2a.plot(t, r, color=COL_REF, linewidth=LW_REF, label='Reference r(t)')
ax2a.plot(t, y_s1, color=COL_RESP1, linewidth=LW_RESP,
          label='Pole placement: {-1,-2,-3,-1.5}')
ax2a.plot(t, y_s2, color=COL_RESP2, linewidth=LW_RESP,
          label=r'LQR ($Q_I$ = 10)')
ax2a.plot(t, y_s3, color=COL_RESP3, linewidth=LW_RESP,
          label=r'LQR ($Q_I$ = 100)')
ax2a.set_xlabel('Time [s]')
ax2a.set_ylabel('Output y(t)')
ax2a.set_title('Effect of Integral Weight on Step Response')
ax2a.legend(loc='best', framealpha=0.9)
ax2a.grid(True, alpha=0.3)

ax2b.plot(t, u_s1, color=COL_RESP1, linewidth=LW_RESP, label='Pole placement')
ax2b.plot(t, u_s2, color=COL_RESP2, linewidth=LW_RESP, label=r'LQR ($Q_I$ = 10)')
ax2b.plot(t, u_s3, color=COL_RESP3, linewidth=LW_RESP, label=r'LQR ($Q_I$ = 100)')
ax2b.set_xlabel('Time [s]')
ax2b.set_ylabel('Control input u(t)')
ax2b.set_title('Control Input Comparison')
ax2b.legend(loc='best', framealpha=0.9)
ax2b.grid(True, alpha=0.3)

plt.tight_layout()
savefig(fig2, 'integral_servo_design_comparison.png')

# ===== Figure 3: Tracking error and integrator state =====
fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=fig_standard)

e_reg = r - y_reg
e_s1 = r - y_s1
e_s2 = r - y_s2
e_s3 = r - y_s3

ax3a.plot(t, e_reg, color=COL_GRAY, linewidth=LW_BASE, linestyle='--',
          label='Regulator (non-zero ss error)')
ax3a.plot(t, e_s1, color=COL_RESP1, linewidth=LW_RESP,
          label='Integral servo (pole placement)')
ax3a.plot(t, e_s2, color=COL_RESP2, linewidth=LW_RESP,
          label=r'Integral servo LQR ($Q_I$ = 10)')
ax3a.plot(t, e_s3, color=COL_RESP3, linewidth=LW_RESP,
          label=r'Integral servo LQR ($Q_I$ = 100)')
ax3a.axhline(0, color='0.7', linewidth=0.5)
ax3a.set_xlabel('Time [s]')
ax3a.set_ylabel('Tracking error e(t) = r - y')
ax3a.set_title('Tracking Error Convergence')
ax3a.legend(loc='best', framealpha=0.9)
ax3a.grid(True, alpha=0.3)

ax3b.plot(t, x_a1_sim[n, :], color=COL_RESP1, linewidth=LW_RESP,
          label='Pole placement')
ax3b.plot(t, x_a2_sim[n, :], color=COL_RESP2, linewidth=LW_RESP,
          label=r'LQR ($Q_I$ = 10)')
ax3b.plot(t, x_a3_sim[n, :], color=COL_RESP3, linewidth=LW_RESP,
          label=r'LQR ($Q_I$ = 100)')
ax3b.set_xlabel('Time [s]')
ax3b.set_ylabel(r'Integrator state $x_I(t)$')
ax3b.set_title('Integrator State')
ax3b.legend(loc='best', framealpha=0.9)
ax3b.grid(True, alpha=0.3)

plt.tight_layout()
savefig(fig3, 'integral_servo_error_integrator.png')

# ===== Figure 4: Pole map =====
fig4, ax4 = plt.subplots(1, 1, figsize=fig_square)

# Open-loop poles
ax4.plot(eig_ol.real, eig_ol.imag, 'x', color=COL_GRAY,
         markersize=12, markeredgewidth=LW_BASE, label='Open-loop poles')

# Regulator poles (no integrator)
eig_reg = np.linalg.eigvals(A - B @ K_reg)
ax4.plot(eig_reg.real, eig_reg.imag, 'o', color=COL_GRAY,
         markersize=10, markeredgewidth=1.5, fillstyle='none',
         label='Regulator (no integrator)')

# Integral servo poles (pole placement)
ax4.plot(eig_s1.real, eig_s1.imag, 's', color=COL_RESP1,
         markersize=10, markeredgewidth=2, fillstyle='none',
         label='Integral servo (pole placement)')

# Integral servo poles (LQR Q_I=10)
ax4.plot(eig_s2.real, eig_s2.imag, 'D', color=COL_RESP2,
         markersize=10, markeredgewidth=2, fillstyle='none',
         label=r'Integral servo LQR ($Q_I$ = 10)')

# Integral servo poles (LQR Q_I=100)
ax4.plot(eig_s3.real, eig_s3.imag, '^', color=COL_RESP3,
         markersize=10, markeredgewidth=2, fillstyle='none',
         label=r'Integral servo LQR ($Q_I$ = 100)')

ax4.axvline(0, color='0.7', linewidth=0.5)
ax4.axhline(0, color='0.7', linewidth=0.5)
ax4.set_xlabel('Real')
ax4.set_ylabel('Imaginary')
ax4.set_title('Pole Map — Integral-Type Servo Designs')
ax4.legend(loc='best', framealpha=0.9)
ax4.grid(True, alpha=0.3)
ax4.set_aspect('equal')

plt.tight_layout()
savefig(fig4, 'integral_servo_pole_map.png')

# ===== Figure 5: Disturbance rejection comparison =====
Tsim_d = 15.0
dt_d = 0.01
t_d = np.arange(0, Tsim_d + dt_d, dt_d)
N_d = len(t_d)
r_d = r_val * np.ones(N_d)

# Regulator + feedforward with disturbance (longer sim)
x_reg_d2 = np.zeros((n, N_d))
u_reg_d2 = np.zeros(N_d)
y_reg_d2 = np.zeros(N_d)
for k in range(N_d - 1):
    y_reg_d2[k] = (C @ x_reg_d2[:, k])[0]
    u_reg_d2[k] = (-K_reg @ x_reg_d2[:, k] + N_bar * r_d[k])[0]
    d_k = d_val if t_d[k] >= t_dist else 0.0
    dx = A @ x_reg_d2[:, k] + B[:, 0] * (u_reg_d2[k] + d_k)
    x_reg_d2[:, k + 1] = x_reg_d2[:, k] + dt_d * dx
y_reg_d2[-1] = (C @ x_reg_d2[:, -1])[0]
u_reg_d2[-1] = (-K_reg @ x_reg_d2[:, -1] + N_bar * r_d[-1])[0]

# Integral servo with disturbance (longer sim)
x_a_d2 = np.zeros((n_a, N_d))
u_s_d2 = np.zeros(N_d)
y_s_d2 = np.zeros(N_d)
for k in range(N_d - 1):
    y_s_d2[k] = (C @ x_a_d2[:n, k])[0]
    u_s_d2[k] = (-K_a2 @ x_a_d2[:, k])[0]
    d_k = d_val if t_d[k] >= t_dist else 0.0
    dx_plant = A @ x_a_d2[:n, k] + B[:, 0] * (u_s_d2[k] + d_k)
    dx_integ = r_d[k] - y_s_d2[k]
    x_a_d2[:, k + 1] = x_a_d2[:, k] + dt_d * np.concatenate([dx_plant, [dx_integ]])
y_s_d2[-1] = (C @ x_a_d2[:n, -1])[0]
u_s_d2[-1] = (-K_a2 @ x_a_d2[:, -1])[0]

fig5, (ax5a, ax5b) = plt.subplots(2, 1, figsize=fig_standard)

ax5a.plot(t_d, r_d, color=COL_REF, linewidth=LW_REF, label='Reference r(t)')
ax5a.plot(t_d, y_reg_d2, color=COL_GRAY, linewidth=LW_BASE, linestyle='--',
          label='Regulator + feedforward')
ax5a.plot(t_d, y_s_d2, color=COL_RESP2, linewidth=LW_RESP,
          label=r'Integral servo LQR ($Q_I$ = 10)')
ax5a.axvline(t_dist, color='0.8', linewidth=1.0, linestyle=':')
ax5a.text(t_dist + 0.1, 0.5, f'Disturbance d = {d_val}\napplied at t = {t_dist}s',
          fontsize=11, color='0.4')
ax5a.set_xlabel('Time [s]')
ax5a.set_ylabel('Output y(t)')
ax5a.set_title('Disturbance Rejection: Regulator vs Integral Servo')
ax5a.legend(loc='best', framealpha=0.9)
ax5a.grid(True, alpha=0.3)

ax5b.plot(t_d, u_reg_d2, color=COL_GRAY, linewidth=LW_BASE, linestyle='--',
          label='Regulator + feedforward')
ax5b.plot(t_d, u_s_d2, color=COL_RESP2, linewidth=LW_RESP,
          label=r'Integral servo LQR ($Q_I$ = 10)')
ax5b.axvline(t_dist, color='0.8', linewidth=1.0, linestyle=':')
ax5b.set_xlabel('Time [s]')
ax5b.set_ylabel('Control input u(t)')
ax5b.set_title('Control Input with Disturbance')
ax5b.legend(loc='best', framealpha=0.9)
ax5b.grid(True, alpha=0.3)

plt.tight_layout()
savefig(fig5, 'integral_servo_disturbance_rejection.png')

# ===== Console summary =====
print()
print("=== Summary ===")
print("Plant: A = [0 1 0; 0 0 1; 1 -1 -1], B = [0;0;1], C = [1 0 0]")
print(f"Reference: step r = {r_val}")
print(f"Regulator ss output: y_ss = {y_reg[-1]:.4f}  (error = {e_reg[-1]:.4f})")
print(f"Integral servo ss output: y_ss = {y_s1[-1]:.4f}  (error = {e_s1[-1]:.4f})")
print("All figures saved to fig/")

plt.show()
