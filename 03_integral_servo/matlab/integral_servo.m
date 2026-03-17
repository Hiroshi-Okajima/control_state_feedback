%% integral_servo.m
%  Integral-Type Servo System for State Feedback Control
%
%  Blog article (Hub): State Feedback Control and State-Space Design
%    https://blog.control-theory.com/entry/state-feedback-control-eng
%  Section: "Integral-Type Servo System"
%
%  Repository: control_state_feedback/03_integral_servo/
%  Author: Hiroshi Okajima, Kumamoto University
%
%  This script demonstrates:
%    1. State feedback WITHOUT integral action (steady-state error remains)
%    2. State feedback WITH integral action (zero steady-state error)
%    3. Effect of integral gain weighting on transient response
%
%  Plant model: Same 3rd-order system as 01_pole_placement/ and 02_lqr_design/
%    A = [0 1 0; 0 0 1; 1 -1 -1],  B = [0; 0; 1],  C = [1 0 0]
%    Open-loop eigenvalues: 0.5437, -0.7719+1.1151j, -0.7719-1.1151j (unstable)

clear; close all; clc;

%% ===== Plot style (self-contained, project standard) =====
set(0, 'DefaultAxesFontName', 'serif');
set(0, 'DefaultAxesFontSize', 13);
set(0, 'DefaultTextFontName', 'serif');
set(0, 'DefaultTextFontSize', 13);
set(0, 'DefaultAxesLabelFontSizeMultiplier', 14/13);
set(0, 'DefaultLegendFontSizeMultiplier', 12/13);

% Color definitions (project standard)
COL_REF    = [0 0 0];             % #000000 - reference
COL_RESP1  = [0.122 0.467 0.706]; % #1f77b4 - response (method 1) blue
COL_RESP2  = [0.839 0.153 0.157]; % #d62728 - response (method 2) red
COL_RESP3  = [0.173 0.627 0.173]; % #2ca02c - response (method 3) green
COL_RESP4  = [1.000 0.498 0.055]; % #ff7f0e - response (method 4) orange
COL_INPUT  = [0.580 0.404 0.741]; % #9467bd - control input
COL_GRAY   = [0.498 0.498 0.498]; % #7f7f7f - baseline / noise

LW_REF  = 2.0;   % linewidth: reference / true
LW_RESP = 1.5;   % linewidth: response / estimate
LW_BASE = 1.5;   % linewidth: baseline (dashed)

fig_standard = [8, 5];   % inches (standard time-series)
fig_square   = [6, 6];   % inches (pole map)
fig_dpi      = 150;

% Create fig/ directory
if ~exist('fig','dir'), mkdir('fig'); end

%% ===== Plant model (same as 01, 02) =====
%  Controllable canonical form, 3rd-order SISO
A = [0 1 0; 0 0 1; 1 -1 -1];
B = [0; 0; 1];
C = [1 0 0];
D = 0;
n = size(A, 1);
m = size(B, 2);   % number of inputs
l = size(C, 1);   % number of outputs

fprintf('=== Plant Model ===\n');
fprintf('Open-loop eigenvalues: ');
fprintf('%.4f%+.4fj  ', [real(eig(A))'; imag(eig(A))']);
fprintf('\n\n');

%% ===== Case 1: State feedback regulator (NO integral action) =====
%  u = -K*x + v  (v is a feedforward term for reference tracking attempt)
%  Steady-state analysis: for step reference r, we need y_ss = r.
%  With u = -Kx, the regulator drives x -> 0, so y -> 0.
%  A naive approach: set v = r (constant feedforward), but this does NOT
%  guarantee y_ss = r in general.

% Design K via pole placement (same poles as 01)
desired_poles_reg = [-1, -2, -3];
K_reg = place(A, B, desired_poles_reg);
fprintf('Regulator gain K_reg = [%.4f  %.4f  %.4f]\n', K_reg);

% Compute feedforward gain for steady-state matching (if possible):
%   y_ss = C * x_ss = r
%   x_ss = (A - B*K_reg)^{-1} * B * v  where  0 = (A-BK)x_ss + B*v
%   => x_ss = -(A-BK)^{-1} * B * v
%   => y_ss = -C*(A-BK)^{-1}*B * v = r
%   => v = -1 / (C*(A-BK)^{-1}*B) * r = N_bar * r
A_cl_reg = A - B*K_reg;
N_bar = -1 / (C * (A_cl_reg \ B));
fprintf('Feedforward gain N_bar = %.4f\n', N_bar);
fprintf('(Note: N_bar provides exact tracking only for nominal plant)\n\n');

%% ===== Case 2: Integral-type servo system =====
%  Augmented state: x_a = [x; x_I], where dot(x_I) = r - y = r - C*x
%  Augmented system: A_a = [A, 0; -C, 0], B_a = [B; 0]
%  Control law: u = -[K_x, K_I] * x_a = -K_x * x - K_I * x_I

A_a = [A, zeros(n, l); -C, zeros(l, l)];
B_a = [B; zeros(l, m)];
n_a = n + l;  % augmented system dimension

fprintf('=== Augmented System (Integral Servo) ===\n');
fprintf('Dimension: n_a = %d (original %d + integrator %d)\n', n_a, n, l);

% Check controllability of augmented system
Uc_a = ctrb(A_a, B_a);
fprintf('Controllability rank of (A_a, B_a): %d (need %d)\n', rank(Uc_a), n_a);

% Check: A must not have eigenvalue at 0 (otherwise augmented loses controllability)
eig_A = eig(A);
if any(abs(eig_A) < 1e-10)
    warning('A has eigenvalue at 0 — augmented system may lose controllability!');
end
fprintf('\n');

%% ===== Design 1: Integral servo via pole placement =====
%  Place n+l = 4 poles
desired_poles_servo = [-1, -2, -3, -1.5];
K_a = place(A_a, B_a, desired_poles_servo);
K_x = K_a(1:n);       % state feedback part
K_I = K_a(n+1:end);   % integral gain part

fprintf('--- Design 1: Pole Placement ---\n');
fprintf('Desired poles: ');
fprintf('%.2f  ', desired_poles_servo);
fprintf('\n');
fprintf('K_x = [%.4f  %.4f  %.4f]\n', K_x);
fprintf('K_I = %.4f\n', K_I);
fprintf('\n');

%% ===== Design 2: Integral servo via LQR (moderate integral weight) =====
Q2 = diag([1, 1, 1, 10]);   % moderate weight on integrator state
R2 = 1;
K_a2 = lqr(A_a, B_a, Q2, R2);
K_x2 = K_a2(1:n);
K_I2 = K_a2(n+1:end);

fprintf('--- Design 2: LQR (Q_I = 10) ---\n');
fprintf('K_x = [%.4f  %.4f  %.4f],  K_I = %.4f\n', K_x2, K_I2);
fprintf('Closed-loop poles: ');
fprintf('%.4f%+.4fj  ', [real(eig(A_a - B_a*K_a2))'; imag(eig(A_a - B_a*K_a2))']);
fprintf('\n\n');

%% ===== Design 3: Integral servo via LQR (strong integral weight) =====
Q3 = diag([1, 1, 1, 100]);  % strong weight on integrator state
R3 = 1;
K_a3 = lqr(A_a, B_a, Q3, R3);
K_x3 = K_a3(1:n);
K_I3 = K_a3(n+1:end);

fprintf('--- Design 3: LQR (Q_I = 100) ---\n');
fprintf('K_x = [%.4f  %.4f  %.4f],  K_I = %.4f\n', K_x3, K_I3);
fprintf('Closed-loop poles: ');
fprintf('%.4f%+.4fj  ', [real(eig(A_a - B_a*K_a3))'; imag(eig(A_a - B_a*K_a3))']);
fprintf('\n\n');

%% ===== Simulation =====
Tsim = 10;     % simulation time [s]
r_val = 1;     % step reference value
dt = 0.01;
t = 0:dt:Tsim;
N = length(t);

% Reference signal: step at t=0
r = r_val * ones(1, N);

% --- Simulate Case 1: Regulator with feedforward (no integrator) ---
% State: x (n states)
% dx/dt = A*x + B*u,  u = -K_reg*x + N_bar*r
x_reg = zeros(n, N);
u_reg = zeros(1, N);
y_reg = zeros(1, N);
x_reg(:,1) = [0; 0; 0];  % zero initial state

for k = 1:N-1
    y_reg(k) = C * x_reg(:,k);
    u_reg(k) = -K_reg * x_reg(:,k) + N_bar * r(k);
    dx = A * x_reg(:,k) + B * u_reg(k);
    x_reg(:,k+1) = x_reg(:,k) + dt * dx;
end
y_reg(N) = C * x_reg(:,N);
u_reg(N) = -K_reg * x_reg(:,N) + N_bar * r(N);

% --- Simulate Case 2: Integral servo (pole placement) ---
x_a1 = zeros(n_a, N);
u_s1 = zeros(1, N);
y_s1 = zeros(1, N);

for k = 1:N-1
    y_s1(k) = C * x_a1(1:n, k);
    u_s1(k) = -K_a * x_a1(:,k);
    dx_plant = A * x_a1(1:n,k) + B * u_s1(k);
    dx_integ = r(k) - y_s1(k);
    x_a1(:,k+1) = x_a1(:,k) + dt * [dx_plant; dx_integ];
end
y_s1(N) = C * x_a1(1:n,N);
u_s1(N) = -K_a * x_a1(:,N);

% --- Simulate Case 3: Integral servo LQR (Q_I = 10) ---
x_a2 = zeros(n_a, N);
u_s2 = zeros(1, N);
y_s2 = zeros(1, N);

for k = 1:N-1
    y_s2(k) = C * x_a2(1:n, k);
    u_s2(k) = -K_a2 * x_a2(:,k);
    dx_plant = A * x_a2(1:n,k) + B * u_s2(k);
    dx_integ = r(k) - y_s2(k);
    x_a2(:,k+1) = x_a2(:,k) + dt * [dx_plant; dx_integ];
end
y_s2(N) = C * x_a2(1:n,N);
u_s2(N) = -K_a2 * x_a2(:,N);

% --- Simulate Case 4: Integral servo LQR (Q_I = 100) ---
x_a3 = zeros(n_a, N);
u_s3 = zeros(1, N);
y_s3 = zeros(1, N);

for k = 1:N-1
    y_s3(k) = C * x_a3(1:n, k);
    u_s3(k) = -K_a3 * x_a3(:,k);
    dx_plant = A * x_a3(1:n,k) + B * u_s3(k);
    dx_integ = r(k) - y_s3(k);
    x_a3(:,k+1) = x_a3(:,k) + dt * [dx_plant; dx_integ];
end
y_s3(N) = C * x_a3(1:n,N);
u_s3(N) = -K_a3 * x_a3(:,N);

%% ===== Figure 1: Step response comparison (with/without integrator) =====
fig1 = figure('Units','inches','Position',[1 1 fig_standard]);

subplot(2,1,1);
plot(t, r, 'Color', COL_REF, 'LineWidth', LW_REF); hold on;
plot(t, y_reg, 'Color', COL_GRAY, 'LineWidth', LW_BASE, 'LineStyle', '--');
plot(t, y_s1, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
xlabel('Time [s]');
ylabel('Output y(t)');
title('Step Response: Regulator vs Integral Servo');
legend('Reference r(t)', ...
       'Regulator + feedforward (no integrator)', ...
       'Integral servo (pole placement)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t, u_reg, 'Color', COL_GRAY, 'LineWidth', LW_BASE, 'LineStyle', '--'); hold on;
plot(t, u_s1, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
xlabel('Time [s]');
ylabel('Control input u(t)');
title('Control Input');
legend('Regulator + feedforward', 'Integral servo', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

set(fig1, 'PaperPositionMode', 'auto');
print(fig1, 'fig/integral_servo_step_comparison.png', '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/integral_servo_step_comparison.png\n');

%% ===== Figure 2: Integral servo with different designs =====
fig2 = figure('Units','inches','Position',[1 1 fig_standard]);

subplot(2,1,1);
plot(t, r, 'Color', COL_REF, 'LineWidth', LW_REF); hold on;
plot(t, y_s1, 'Color', COL_RESP1, 'LineWidth', LW_RESP);
plot(t, y_s2, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
plot(t, y_s3, 'Color', COL_RESP3, 'LineWidth', LW_RESP);
xlabel('Time [s]');
ylabel('Output y(t)');
title('Effect of Integral Weight on Step Response');
legend('Reference r(t)', ...
       'Pole placement: \{-1,-2,-3,-1.5\}', ...
       'LQR (Q_I = 10)', ...
       'LQR (Q_I = 100)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t, u_s1, 'Color', COL_RESP1, 'LineWidth', LW_RESP); hold on;
plot(t, u_s2, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
plot(t, u_s3, 'Color', COL_RESP3, 'LineWidth', LW_RESP);
xlabel('Time [s]');
ylabel('Control input u(t)');
title('Control Input Comparison');
legend('Pole placement', 'LQR (Q_I = 10)', 'LQR (Q_I = 100)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

set(fig2, 'PaperPositionMode', 'auto');
print(fig2, 'fig/integral_servo_design_comparison.png', '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/integral_servo_design_comparison.png\n');

%% ===== Figure 3: Tracking error and integrator state =====
fig3 = figure('Units','inches','Position',[1 1 fig_standard]);

% Tracking error
subplot(2,1,1);
e_reg = r - y_reg;
e_s1  = r - y_s1;
e_s2  = r - y_s2;
e_s3  = r - y_s3;

plot(t, e_reg, 'Color', COL_GRAY, 'LineWidth', LW_BASE, 'LineStyle', '--'); hold on;
plot(t, e_s1, 'Color', COL_RESP1, 'LineWidth', LW_RESP);
plot(t, e_s2, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
plot(t, e_s3, 'Color', COL_RESP3, 'LineWidth', LW_RESP);
yline(0, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
xlabel('Time [s]');
ylabel('Tracking error e(t) = r - y');
title('Tracking Error Convergence');
legend('Regulator (non-zero ss error)', ...
       'Integral servo (pole placement)', ...
       'Integral servo LQR (Q_I = 10)', ...
       'Integral servo LQR (Q_I = 100)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

% Integrator state
subplot(2,1,2);
plot(t, x_a1(n+1,:), 'Color', COL_RESP1, 'LineWidth', LW_RESP); hold on;
plot(t, x_a2(n+1,:), 'Color', COL_RESP2, 'LineWidth', LW_RESP);
plot(t, x_a3(n+1,:), 'Color', COL_RESP3, 'LineWidth', LW_RESP);
xlabel('Time [s]');
ylabel('Integrator state x_I(t)');
title('Integrator State');
legend('Pole placement', 'LQR (Q_I = 10)', 'LQR (Q_I = 100)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

set(fig3, 'PaperPositionMode', 'auto');
print(fig3, 'fig/integral_servo_error_integrator.png', '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/integral_servo_error_integrator.png\n');

%% ===== Figure 4: Pole map =====
fig4 = figure('Units','inches','Position',[1 1 fig_square]);

% Open-loop poles
eig_ol = eig(A);
plot(real(eig_ol), imag(eig_ol), 'x', 'Color', COL_GRAY, ...
     'MarkerSize', 12, 'LineWidth', LW_BASE); hold on;

% Regulator poles (no integrator)
eig_reg = eig(A - B*K_reg);
plot(real(eig_reg), imag(eig_reg), 'o', 'Color', COL_GRAY, ...
     'MarkerSize', 10, 'LineWidth', 1.5);

% Integral servo poles (pole placement)
eig_s1 = eig(A_a - B_a*K_a);
plot(real(eig_s1), imag(eig_s1), 's', 'Color', COL_RESP1, ...
     'MarkerSize', 10, 'LineWidth', 2);

% Integral servo poles (LQR Q_I=10)
eig_s2 = eig(A_a - B_a*K_a2);
plot(real(eig_s2), imag(eig_s2), 'd', 'Color', COL_RESP2, ...
     'MarkerSize', 10, 'LineWidth', 2);

% Integral servo poles (LQR Q_I=100)
eig_s3 = eig(A_a - B_a*K_a3);
plot(real(eig_s3), imag(eig_s3), '^', 'Color', COL_RESP3, ...
     'MarkerSize', 10, 'LineWidth', 2);

xline(0, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
yline(0, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);

xlabel('Real');
ylabel('Imaginary');
title('Pole Map — Integral-Type Servo Designs');
legend('Open-loop poles', ...
       'Regulator (no integrator)', ...
       'Integral servo (pole placement)', ...
       'Integral servo LQR (Q_I = 10)', ...
       'Integral servo LQR (Q_I = 100)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);
axis equal;

print(fig4, 'fig/integral_servo_pole_map.png', '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/integral_servo_pole_map.png\n');

%% ===== Figure 5: Disturbance rejection =====
%  Step disturbance d = 0.5 applied at t = 5 s
d_val = 0.5;
t_dist = 5;

Tsim_d = 15;
t_d = 0:dt:Tsim_d;
N_d = length(t_d);
r_d = r_val * ones(1, N_d);

% Regulator + feedforward with disturbance
x_reg_d = zeros(n, N_d);
u_reg_d = zeros(1, N_d);
y_reg_d = zeros(1, N_d);
for k = 1:N_d-1
    y_reg_d(k) = C * x_reg_d(:,k);
    u_reg_d(k) = -K_reg * x_reg_d(:,k) + N_bar * r_d(k);
    d_k = d_val * (t_d(k) >= t_dist);
    dx = A * x_reg_d(:,k) + B * (u_reg_d(k) + d_k);
    x_reg_d(:,k+1) = x_reg_d(:,k) + dt * dx;
end
y_reg_d(N_d) = C * x_reg_d(:,N_d);

% Integral servo LQR (Q_I=10) with disturbance
x_a_d = zeros(n_a, N_d);
u_s_d = zeros(1, N_d);
y_s_d = zeros(1, N_d);
for k = 1:N_d-1
    y_s_d(k) = C * x_a_d(1:n,k);
    u_s_d(k) = -K_a2 * x_a_d(:,k);
    d_k = d_val * (t_d(k) >= t_dist);
    dx_plant = A * x_a_d(1:n,k) + B * (u_s_d(k) + d_k);
    dx_integ = r_d(k) - y_s_d(k);
    x_a_d(:,k+1) = x_a_d(:,k) + dt * [dx_plant; dx_integ];
end
y_s_d(N_d) = C * x_a_d(1:n,N_d);

fig5 = figure('Units','inches','Position',[1 1 fig_standard]);

subplot(2,1,1);
plot(t_d, r_d, 'Color', COL_REF, 'LineWidth', LW_REF); hold on;
plot(t_d, y_reg_d, 'Color', COL_GRAY, 'LineWidth', LW_BASE, 'LineStyle', '--');
plot(t_d, y_s_d, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
xline(t_dist, ':', 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0);
text(t_dist+0.2, 0.5, sprintf('Disturbance d = %.1f\napplied at t = %ds', d_val, t_dist), ...
     'FontSize', 11, 'Color', [0.4 0.4 0.4]);
xlabel('Time [s]');
ylabel('Output y(t)');
title('Disturbance Rejection: Regulator vs Integral Servo');
legend('Reference r(t)', 'Regulator + feedforward', ...
       'Integral servo LQR (Q_I = 10)', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t_d, u_reg_d, 'Color', COL_GRAY, 'LineWidth', LW_BASE, 'LineStyle', '--'); hold on;
plot(t_d, u_s_d, 'Color', COL_RESP2, 'LineWidth', LW_RESP);
xline(t_dist, ':', 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Control input u(t)');
title('Control Input with Disturbance');
legend('Regulator + feedforward', 'Integral servo LQR (Q_I = 10)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

set(fig5, 'PaperPositionMode', 'auto');
print(fig5, 'fig/integral_servo_disturbance_rejection.png', '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/integral_servo_disturbance_rejection.png\n');

%% ===== Console summary =====
fprintf('\n=== Summary ===\n');
fprintf('Plant: A = [0 1 0; 0 0 1; 1 -1 -1], B = [0;0;1], C = [1 0 0]\n');
fprintf('Reference: step r = %.1f\n', r_val);
fprintf('Regulator (no integrator): steady-state error exists\n');
fprintf('Integral servo (3 designs): zero steady-state error for step input\n');
fprintf('All figures saved to fig/\n');
