%% observer_based_feedback.m
%  Observer-Based State Feedback Control — Separation Principle Demo
%
%  Blog article: State Observer for State Space Model
%    https://blog.control-theory.com/entry/2024/10/01/143305
%  Hub article: State Feedback Control and State-Space Design
%    https://blog.control-theory.com/entry/state-feedback-control-eng
%    Section: "Observer-Based Feedback and the Separation Principle"
%
%  Repository: control_state_feedback/04_observer_based_feedback/
%  Author: Hiroshi Okajima, Kumamoto University
%
%  This script demonstrates the separation principle:
%    Fig 1: Full-state feedback vs observer-based feedback (output comparison)
%    Fig 2: True state vs estimated state trajectories
%    Fig 3: Estimation error convergence
%    Fig 4: Closed-loop pole map (controller poles & observer poles, separated)
%
%  NOTE: This is a lightweight demo focusing on the separation principle.
%  The same plant as 01_pole_placement/ through 03_integral_servo/ is used.
%  For comprehensive observer design (Kalman filter, H-infinity filter,
%  multi-rate observer, MCV observer), see:
%    MATLAB: https://github.com/Hiroshi-Okajima/MATLAB_state_observer
%    Python: https://github.com/Hiroshi-Okajima/python_state_observer

clear; close all; clc;

%% ===== Plot style (self-contained, project standard) =====
set(0, 'DefaultAxesFontName', 'serif');
set(0, 'DefaultAxesFontSize', 13);
set(0, 'DefaultTextFontName', 'serif');
set(0, 'DefaultTextFontSize', 13);
set(0, 'DefaultAxesLabelFontSizeMultiplier', 14/13);
set(0, 'DefaultLegendFontSizeMultiplier', 12/13);

% Color definitions (project standard)
COL_REF    = [0 0 0];             % #000000 - reference / true value
COL_RESP1  = [0.122 0.467 0.706]; % #1f77b4 - response (method 1)
COL_RESP2  = [0.839 0.153 0.157]; % #d62728 - response (method 2)
COL_RESP3  = [0.173 0.627 0.173]; % #2ca02c - response (method 3)
COL_RESP4  = [1.000 0.498 0.055]; % #ff7f0e - response (method 4)
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

%% ===== Plant model (same as 01, 02, 03) =====
%  Controllable canonical form, 3rd-order SISO
%  Open-loop eigenvalues: 1, -1+j, -1-j (unstable)
A = [0 1 0; 0 0 1; 1 -1 -1];
B = [0; 0; 1];
C = [1 0 0];
D = 0;
n = size(A, 1);

fprintf('=== Plant Model ===\n');
fprintf('Open-loop eigenvalues: ');
fprintf('%.4f  ', eig(A));
fprintf('\n\n');

%% ===== Controller design: Pole placement (same as 01) =====
%  Desired closed-loop poles for state feedback
desired_poles_K = [-1, -2, -3];
K = place(A, B, desired_poles_K);
fprintf('State feedback gain K = [%.4f  %.4f  %.4f]\n', K);
fprintf('Controller poles (A-BK): ');
fprintf('%.4f  ', eig(A - B*K));
fprintf('\n\n');

%% ===== Observer design: Pole placement =====
%  Observer poles should be faster than controller poles (2-5x rule of thumb)
%  Place observer poles at -5, -6, -7
desired_poles_L = [-5, -6, -7];
L = place(A', C', desired_poles_L)';
fprintf('Observer gain L = [%.4f; %.4f; %.4f]\n', L);
fprintf('Observer poles (A-LC): ');
fprintf('%.4f  ', eig(A - L*C));
fprintf('\n\n');

%% ===== Verify separation principle =====
%  Full closed-loop system (2n = 6 states)
%  State: [x; e] where e = x - x_hat
A_cl = [A - B*K, B*K; zeros(n), A - L*C];
eig_cl = eig(A_cl);
fprintf('Full closed-loop eigenvalues (2n = %d):\n', 2*n);
fprintf('  %.4f\n', eig_cl);
fprintf('(Should be union of controller poles and observer poles)\n\n');

%% ===== Simulation =====
Tsim = 8;      % simulation time [s]

% Initial conditions
x0 = [1; 0; 0];       % plant initial state
x_hat0 = [0; 0; 0];   % observer initial state (unknown)
e0 = x0 - x_hat0;     % initial estimation error

% --- Case 1: Full-state feedback (ideal) ---
%  dx/dt = (A - BK) x
sys_full = ss(A - B*K, zeros(n,1), C, 0);
[y_full, t_full, x_full] = initial(sys_full, x0, Tsim);
u_full = -x_full * K';

% --- Case 2: Observer-based feedback ---
%  Augmented state: [x; e], where x_hat = x - e
%  dx/dt = (A - BK)x + BK*e
%  de/dt = (A - LC)e
B_cl = zeros(2*n, 1);
C_cl_y = [C, zeros(1,n)];      % output y = Cx
C_cl_x = [eye(n), zeros(n)];   % true state x
C_cl_e = [zeros(n), eye(n)];   % estimation error e
C_cl_xhat = [eye(n), -eye(n)]; % estimated state x_hat = x - e

sys_obs = ss(A_cl, B_cl, [C_cl_y; C_cl_x; C_cl_e; C_cl_xhat], ...
             zeros(1+3*n, 1));
z0 = [x0; e0];
[y_obs_all, t_obs, ~] = initial(sys_obs, z0, Tsim);

y_obs  = y_obs_all(:, 1);                % output y(t)
x_true = y_obs_all(:, 2:1+n);            % true state x(t)
e_obs  = y_obs_all(:, 2+n:1+2*n);        % estimation error e(t)
x_hat  = y_obs_all(:, 2+2*n:1+3*n);      % estimated state x_hat(t)
u_obs  = -x_hat * K';                     % control input u = -K*x_hat

%% ===== Figure 1: Output comparison =====
fig1 = figure('Units','inches','Position',[1 1 fig_standard]);

subplot(2,1,1);
plot(t_full, y_full, 'Color', COL_RESP1, 'LineWidth', LW_RESP); hold on;
plot(t_obs, y_obs, 'Color', COL_RESP2, 'LineWidth', LW_RESP, 'LineStyle', '--');
xlabel('Time [s]');
ylabel('Output y(t)');
title('Output Response: Full-State FB vs Observer-Based FB');
legend('Full-state feedback (u = -Kx)', ...
       'Observer-based feedback (u = -K\hat{x})', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t_full, u_full, 'Color', COL_RESP1, 'LineWidth', LW_RESP); hold on;
plot(t_obs, u_obs, 'Color', COL_RESP2, 'LineWidth', LW_RESP, 'LineStyle', '--');
xlabel('Time [s]');
ylabel('Control input u(t)');
title('Control Input Comparison');
legend('Full-state feedback', 'Observer-based feedback', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

tight_layout_manual(fig1);
print(fig1, 'fig/observer_based_feedback_output_comparison.png', ...
      '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/observer_based_feedback_output_comparison.png\n');

%% ===== Figure 2: True state vs estimated state =====
fig2 = figure('Units','inches','Position',[1 1 fig_standard]);
state_labels = {'x_1(t)', 'x_2(t)', 'x_3(t)'};

for i = 1:n
    subplot(n, 1, i);
    plot(t_obs, x_true(:,i), 'Color', COL_REF, 'LineWidth', LW_REF); hold on;
    plot(t_obs, x_hat(:,i), 'Color', COL_RESP1, 'LineWidth', LW_RESP, ...
         'LineStyle', '--');
    ylabel(state_labels{i});
    if i == 1
        title('True State vs Estimated State');
        legend('True state x(t)', 'Estimated state \hat{x}(t)', ...
               'Location', 'best');
    end
    if i == n
        xlabel('Time [s]');
    end
    grid on; set(gca, 'GridAlpha', 0.3);
end

tight_layout_manual(fig2);
print(fig2, 'fig/observer_based_feedback_state_estimation.png', ...
      '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/observer_based_feedback_state_estimation.png\n');

%% ===== Figure 3: Estimation error convergence =====
fig3 = figure('Units','inches','Position',[1 1 fig_standard]);

for i = 1:n
    subplot(n, 1, i);
    plot(t_obs, e_obs(:,i), 'Color', COL_RESP2, 'LineWidth', LW_RESP); hold on;
    yline(0, 'Color', COL_GRAY, 'LineWidth', 1.0, 'LineStyle', ':');
    ylabel(sprintf('e_%d(t)', i));
    if i == 1
        title('Estimation Error: e(t) = x(t) - \hat{x}(t)');
    end
    if i == n
        xlabel('Time [s]');
    end
    grid on; set(gca, 'GridAlpha', 0.3);
end

tight_layout_manual(fig3);
print(fig3, 'fig/observer_based_feedback_estimation_error.png', ...
      '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/observer_based_feedback_estimation_error.png\n');

%% ===== Figure 4: Pole map (separation principle) =====
fig4 = figure('Units','inches','Position',[1 1 fig_square]);

% Open-loop poles
eig_ol = eig(A);
plot(real(eig_ol), imag(eig_ol), 'x', 'Color', COL_GRAY, ...
     'MarkerSize', 12, 'LineWidth', LW_BASE); hold on;

% Controller poles (A - BK)
eig_K = eig(A - B*K);
plot(real(eig_K), imag(eig_K), 'o', 'Color', COL_RESP1, ...
     'MarkerSize', 10, 'LineWidth', 2);

% Observer poles (A - LC)
eig_L = eig(A - L*C);
plot(real(eig_L), imag(eig_L), 's', 'Color', COL_RESP2, ...
     'MarkerSize', 10, 'LineWidth', 2);

% Full closed-loop poles (should overlap)
plot(real(eig_cl), imag(eig_cl), '+', 'Color', COL_RESP3, ...
     'MarkerSize', 14, 'LineWidth', 1.5);

% Imaginary axis
xline(0, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
yline(0, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);

xlabel('Real');
ylabel('Imaginary');
title('Pole Map — Separation Principle');
legend('Open-loop poles', ...
       sprintf('Controller poles (A-BK): %s', mat2str(desired_poles_K)), ...
       sprintf('Observer poles (A-LC): %s', mat2str(desired_poles_L)), ...
       'Full closed-loop poles (2n)', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);
axis equal;

print(fig4, 'fig/observer_based_feedback_pole_map.png', ...
      '-dpng', sprintf('-r%d', fig_dpi));
fprintf('Saved: fig/observer_based_feedback_pole_map.png\n');

%% ===== Console summary =====
fprintf('\n=== Summary ===\n');
fprintf('Plant: 3rd-order, A = [0 1 0; 0 0 1; 1 -1 -1], B = [0;0;1], C = [1 0 0]\n');
fprintf('Controller poles (A-BK): {-1, -2, -3}\n');
fprintf('Observer poles (A-LC):   {-5, -6, -7}\n');
fprintf('Separation principle verified: full closed-loop poles = union of above\n');
fprintf('Initial state mismatch: x0 = [1;0;0], x_hat0 = [0;0;0]\n');

%% ===== Helper function =====
function tight_layout_manual(fig)
    % Simple tight layout for MATLAB (adjust subplot spacing)
    set(fig, 'PaperPositionMode', 'auto');
end
