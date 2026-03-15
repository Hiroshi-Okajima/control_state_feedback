
%% lqr_design.m
%  State Feedback Control: LQR (Optimal Regulator) Design
%
%  Blog article:
%    https://blog.control-theory.com/entry/2024/10/01/143503
%  Hub article:
%    https://blog.control-theory.com/entry/state-feedback-control-eng
%
%  Repository: control_state_feedback/02_lqr_design/
%
%  This script demonstrates:
%    1. LQR response comparison with different R weights (Q = I fixed)
%    2. State and control input trade-off visualization
%    3. Riccati equation solution convergence
%
%  Plant model (same 3rd-order system as 01_pole_placement):
%    A = [0 1 0; 0 0 1; -1 1 1],  B = [0; 0; 1],  C = [1 0 0], D = 0
%    Open-loop poles: 1 (double), -1  (unstable)

clear; close all; clc;

%% --- Plot style (self-contained) ---
colors.reference = [0.000 0.000 0.000];   % #000000  black
colors.method1   = [0.122 0.467 0.706];   % #1f77b4  blue
colors.method2   = [0.839 0.153 0.157];   % #d62728  red
colors.method3   = [0.173 0.627 0.173];   % #2ca02c  green
colors.method4   = [1.000 0.498 0.055];   % #ff7f0e  orange
colors.control   = [0.580 0.404 0.741];   % #9467bd  purple
colors.noise     = [0.498 0.498 0.498];   % #7f7f7f  gray
colors.baseline  = [0.498 0.498 0.498];   % #7f7f7f  gray
figsize.standard = [8, 5];
figsize.wide     = [8, 6];
figsize.square   = [6, 6];
lw.reference = 2.0;  lw.response = 1.5;  lw.baseline = 1.5;  lw.noise = 1.0;
set(0, 'DefaultAxesFontSize', 13, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 13, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultLegendFontSize', 12);

%% --- Create output directory ---
if ~exist('fig', 'dir'), mkdir('fig'); end

%% =====================================================================
%%  Plant definition (same as 01_pole_placement)
%% =====================================================================
A = [0  1  0;
     0  0  1;
    -1  1  1];
B = [0; 0; 1];
C = [1  0  0];
D = 0;

n = size(A,1);

fprintf('Open-loop eigenvalues of A:\n');
disp(eig(A).');

%% =====================================================================
%%  LQR designs with different R weights  (Q = I fixed)
%% =====================================================================
Q = eye(n);

R_vals = [0.01, 1, 100];
R_labels = {'R = 0.01 (fast)', 'R = 1 (balanced)', 'R = 100 (energy-saving)'};
R_colors = {colors.method1, colors.method2, colors.method3};

K_all = zeros(length(R_vals), n);
P_all = cell(length(R_vals), 1);
J_all = zeros(length(R_vals), 1);

for i = 1:length(R_vals)
    [K_all(i,:), P_all{i}] = lqr(A, B, Q, R_vals(i));
    Acl = A - B*K_all(i,:);
    fprintf('R = %g:  K = [%s],  CL poles = [%s]\n', ...
        R_vals(i), num2str(K_all(i,:), '%.4f '), ...
        num2str(real(eig(Acl)).', '%.4f '));
end

%% =====================================================================
%%  Simulation parameters
%% =====================================================================
x0 = [1; 0; 0];
Tsim = 8;
dt = 0.001;
t = 0:dt:Tsim;

%% --- Simulate all designs ---
X_all = cell(length(R_vals), 1);
U_all = cell(length(R_vals), 1);
Y_all = cell(length(R_vals), 1);

for i = 1:length(R_vals)
    [Xtmp, Utmp] = sim_state_fb(A, B, K_all(i,:), x0, t);
    X_all{i} = Xtmp;
    U_all{i} = Utmp;
    Y_all{i} = C * Xtmp;
end

%% =====================================================================
%%  Figure 1: Output response comparison (different R)
%% =====================================================================
fig1 = figure('Units','inches','Position',[1 1 figsize.standard]);

subplot(2,1,1);
for i = 1:length(R_vals)
    plot(t, Y_all{i}, 'Color', R_colors{i}, 'LineWidth', lw.response); hold on;
end
yline(0, 'Color', colors.reference, 'LineWidth', 0.8, 'LineStyle', '--');
xlabel('Time [s]');
ylabel('Output  y = x_1');
title('LQR Response: Effect of R Weight (Q = I)');
legend(R_labels{:}, 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
for i = 1:length(R_vals)
    plot(t, U_all{i}, 'Color', R_colors{i}, 'LineWidth', lw.response); hold on;
end
xlabel('Time [s]');
ylabel('Control input  u');
title('Control Input');
legend(R_labels{:}, 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

save_figure(fig1, 'fig/lqr_design_R_comparison.png');

%% =====================================================================
%%  Figure 2: All three state variables (for R = 0.01 and R = 100)
%% =====================================================================
fig2 = figure('Units','inches','Position',[1 1 figsize.wide]);
state_labels = {'x_1', 'x_2', 'x_3'};

for j = 1:3
    subplot(3,1,j);
    plot(t, X_all{1}(j,:), 'Color', colors.method1, 'LineWidth', lw.response); hold on;
    plot(t, X_all{3}(j,:), 'Color', colors.method3, 'LineWidth', lw.response);
    yline(0, 'Color', colors.reference, 'LineWidth', 0.8, 'LineStyle', '--');
    ylabel(state_labels{j});
    grid on; set(gca, 'GridAlpha', 0.3);
    if j == 1
        title('State Variables: R = 0.01 vs R = 100');
        legend('R = 0.01', 'R = 100', 'Location', 'best');
    end
    if j == 3
        xlabel('Time [s]');
    end
end

save_figure(fig2, 'fig/lqr_design_state_variables.png');

%% =====================================================================
%%  Figure 3: Pole map comparison (LQR vs pole placement from 01)
%% =====================================================================
fig3 = figure('Units','inches','Position',[1 1 figsize.square]);

ol_poles = eig(A);
plot(real(ol_poles), imag(ol_poles), 'kx', 'MarkerSize', 12, 'LineWidth', 2.0);
hold on;

markers = {'o', 's', 'd'};
for i = 1:length(R_vals)
    Acl = A - B*K_all(i,:);
    cl_poles = eig(Acl);
    plot(real(cl_poles), imag(cl_poles), markers{i}, ...
        'Color', R_colors{i}, 'MarkerSize', 10, 'LineWidth', 1.5, ...
        'MarkerFaceColor', 'none');
end

% Add pole placement Design 1 from 01 for reference
p_pp = [-1, -2, -3];
K_pp = acker(A, B, p_pp);
Acl_pp = A - B*K_pp;
cl_pp = eig(Acl_pp);
plot(real(cl_pp), imag(cl_pp), '^', 'Color', colors.method4, ...
     'MarkerSize', 10, 'LineWidth', 1.5, 'MarkerFaceColor', 'none');

xline(0, 'k--', 'LineWidth', 0.8);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Real');
ylabel('Imaginary');
title('Pole Map: LQR Designs vs Pole Placement');
legend('Open-loop', R_labels{:}, 'Pole place \{-1,-2,-3\}', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);
axis equal;

save_figure(fig3, 'fig/lqr_design_pole_map.png');

%% =====================================================================
%%  Figure 4: Cost function trade-off  (J vs R)
%% =====================================================================
R_sweep = logspace(-3, 3, 50);
J_state = zeros(size(R_sweep));
J_input = zeros(size(R_sweep));

for i = 1:length(R_sweep)
    [Ktmp, Ptmp] = lqr(A, B, Q, R_sweep(i));
    % J = x0' * P * x0  (total cost for given x0)
    % Decompose: J_state ~ integral x'Qx, J_input ~ integral u'Ru
    % For analysis, compute via simulation
    [Xtmp, Utmp] = sim_state_fb(A, B, Ktmp, x0, t);
    J_state(i) = dt * sum(sum((Q * Xtmp) .* Xtmp, 1));
    J_input(i) = dt * R_sweep(i) * sum(Utmp.^2);
end

fig4 = figure('Units','inches','Position',[1 1 figsize.standard]);

subplot(2,1,1);
semilogx(R_sweep, J_state, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
semilogx(R_sweep, J_input, 'Color', colors.method2, 'LineWidth', lw.response);
semilogx(R_sweep, J_state + J_input, 'Color', colors.reference, ...
    'LineWidth', lw.reference, 'LineStyle', '--');
xlabel('Weight R');
ylabel('Cost');
title('Cost Function Decomposition vs R Weight');
legend('\int x^T Q x dt', '\int u^T R u dt', 'Total J', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
semilogx(R_sweep, J_state, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
semilogx(R_sweep, J_input ./ R_sweep, 'Color', colors.control, 'LineWidth', lw.response);
xlabel('Weight R');
ylabel('Value');
title('State Cost vs Input Energy (\int u^2 dt)');
legend('\int x^T Q x dt  (state penalty)', ...
       '\int u^2 dt  (input energy)', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

save_figure(fig4, 'fig/lqr_design_cost_tradeoff.png');

fprintf('\nAll figures saved to fig/ folder.\n');

%% =====================================================================
%%  Local function: state feedback simulation (RK4)
%% =====================================================================
function [X, U] = sim_state_fb(A, B, K, x0, t)
    n_state = length(x0);
    Nt = length(t);
    dt = t(2) - t(1);
    X = zeros(n_state, Nt);
    U = zeros(1, Nt);
    X(:,1) = x0;
    U(1) = -K * x0;
    for k = 1:Nt-1
        xk = X(:,k);
        uk = -K * xk;
        U(k) = uk;
        f1 = A*xk + B*uk;
        f2 = A*(xk + dt/2*f1) + B*(-K*(xk + dt/2*f1));
        f3 = A*(xk + dt/2*f2) + B*(-K*(xk + dt/2*f2));
        f4 = A*(xk + dt*f3)   + B*(-K*(xk + dt*f3));
        X(:,k+1) = xk + dt/6*(f1 + 2*f2 + 2*f3 + f4);
    end
    U(Nt) = -K * X(:,Nt);
end

function save_figure(fig, filepath)
    exportgraphics(fig, filepath, 'Resolution', 150, ...
                   'BackgroundColor', 'white');
    fprintf('Saved: %s\n', filepath);
end
