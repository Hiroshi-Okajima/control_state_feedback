%% pole_placement.m
%  State Feedback Control: Pole Placement Simulation
%
%  Blog article:
%    https://blog.control-theory.com/entry/2024/10/01/143612
%  Hub article:
%    https://blog.control-theory.com/entry/state-feedback-control-eng
%
%  Repository: control_state_feedback/01_pole_placement/
%
%  This script demonstrates:
%    1. Step response comparison — open-loop (unstable) vs closed-loop
%       with different pole placements
%    2. Pole map on the s-plane
%    3. Effect of pole locations on response speed and oscillation
%
%  Plant model (3rd-order controllable canonical form from the blog):
%    A = [0 1 0; 0 0 1; -1 1 1],  B = [0; 0; 1],  C = [1 0 0], D = 0
%    Open-loop poles: 1 (double), -1  (unstable)

clear; close all; clc;

%% --- Plot style (self-contained — no external dependency) ---
colors.reference = [0.000 0.000 0.000];   % #000000  black
colors.method1   = [0.122 0.467 0.706];   % #1f77b4  blue
colors.method2   = [0.839 0.153 0.157];   % #d62728  red
colors.method3   = [0.173 0.627 0.173];   % #2ca02c  green
colors.method4   = [1.000 0.498 0.055];   % #ff7f0e  orange
colors.control   = [0.580 0.404 0.741];   % #9467bd  purple
colors.noise     = [0.498 0.498 0.498];   % #7f7f7f  gray
colors.baseline  = [0.498 0.498 0.498];   % #7f7f7f  gray
figsize.standard = [8, 5];    % 1200 x 750 px at 150 dpi
figsize.wide     = [8, 6];    % 1200 x 900 px
figsize.square   = [6, 6];    %  900 x 900 px
lw.reference = 2.0;  lw.response = 1.5;  lw.baseline = 1.5;  lw.noise = 1.0;
set(0, 'DefaultAxesFontSize', 13, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 13, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultLegendFontSize', 12);

%% =====================================================================
%%  Plant definition
%% =====================================================================
A = [0  1  0;
     0  0  1;
    -1  1  1];
B = [0; 0; 1];
C = [1  0  0];
D = 0;

n = size(A,1);  % system order

fprintf('Open-loop eigenvalues of A:\n');
disp(eig(A));

%% =====================================================================
%%  Design 1: Gain K1 — poles at {-1, -2, -3}  (from blog article)
%% =====================================================================
p1 = [-1, -2, -3];
K1 = acker(A, B, p1);
Acl1 = A - B*K1;
fprintf('Design 1: K = [%s]\n', num2str(K1, '%.4f '));
fprintf('  Closed-loop poles: ');  disp(eig(Acl1).');

%% =====================================================================
%%  Design 2: Gain K2 — poles at {-0.5, -1, -2}  (slow, near imag axis)
%% =====================================================================
p2 = [-0.5, -1, -2];
K2 = acker(A, B, p2);
Acl2 = A - B*K2;

%% =====================================================================
%%  Design 3: Gain K3 — poles at {-5, -6, -7}  (fast, far left)
%% =====================================================================
p3 = [-5, -6, -7];
K3 = acker(A, B, p3);
Acl3 = A - B*K3;

%% =====================================================================
%%  Design 4: Gain K4 — poles at {-2, -1+5i, -1-5i}  (oscillatory)
%% =====================================================================
p4 = [-2, -1+5i, -1-5i];
K4 = acker(A, B, p4);
Acl4 = A - B*K4;

%% =====================================================================
%%  Simulation parameters
%% =====================================================================
x0 = [1; 0; 0];          % initial state
Tsim = 8;                 % simulation time [s]
dt = 0.001;               % integration step
t = 0:dt:Tsim;
Nt = length(t);

%% --- Simulate all designs (Euler integration) ---
simulate = @(Acl) sim_state(Acl, x0, t);

[X1, U1] = sim_state_fb(A, B, K1, x0, t);
[X2, U2] = sim_state_fb(A, B, K2, x0, t);
[X3, U3] = sim_state_fb(A, B, K3, x0, t);
[X4, U4] = sim_state_fb(A, B, K4, x0, t);

% Output y = Cx
Y1 = (C * X1)';
Y2 = (C * X2)';
Y3 = (C * X3)';
Y4 = (C * X4)';

%% =====================================================================
%%  Figure 1: Step response comparison (Design 1, 2, 3)
%% =====================================================================
fig1 = figure('Units','inches','Position',[1 1 figsize.standard]);

subplot(2,1,1);
plot(t, Y1, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
plot(t, Y2, 'Color', colors.method4, 'LineWidth', lw.response);
plot(t, Y3, 'Color', colors.method3, 'LineWidth', lw.response);
yline(0, 'Color', colors.reference, 'LineWidth', 0.8, 'LineStyle', '--');
xlabel('Time [s]');
ylabel('Output y = x_1');
title('State Response: Effect of Real-Part Pole Locations');
legend(sprintf('Design 1: poles = {%s}', num2str(p1, '%g, ')), ...
       sprintf('Design 2: poles = {%s}', num2str(p2, '%g, ')), ...
       sprintf('Design 3: poles = {%s}', num2str(p3, '%g, ')), ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t, U1, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
plot(t, U2, 'Color', colors.method4, 'LineWidth', lw.response);
plot(t, U3, 'Color', colors.method3, 'LineWidth', lw.response);
xlabel('Time [s]');
ylabel('Control input u');
title('Control Input');
legend('Design 1', 'Design 2', 'Design 3', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

save_figure(fig1, 'fig/pole_placement_step_response.png');

%% =====================================================================
%%  Figure 2: Pole map on s-plane
%% =====================================================================
fig2 = figure('Units','inches','Position',[1 1 figsize.square]);

ol_poles = eig(A);
cl1_poles = eig(Acl1);
cl2_poles = eig(Acl2);
cl3_poles = eig(Acl3);
cl4_poles = eig(Acl4);

plot(real(ol_poles), imag(ol_poles), 'kx', 'MarkerSize', 12, ...
     'LineWidth', 2.0); hold on;
plot(real(cl1_poles), imag(cl1_poles), 'o', 'Color', colors.method1, ...
     'MarkerSize', 10, 'LineWidth', 1.5);
plot(real(cl2_poles), imag(cl2_poles), 's', 'Color', colors.method4, ...
     'MarkerSize', 10, 'LineWidth', 1.5);
plot(real(cl3_poles), imag(cl3_poles), 'd', 'Color', colors.method3, ...
     'MarkerSize', 10, 'LineWidth', 1.5);
plot(real(cl4_poles), imag(cl4_poles), '^', 'Color', colors.method2, ...
     'MarkerSize', 10, 'LineWidth', 1.5);

xline(0, 'k--', 'LineWidth', 0.8);
yline(0, 'k--', 'LineWidth', 0.8);

xlabel('Real');
ylabel('Imaginary');
title('Pole Map (s-plane)');
legend('Open-loop', 'Design 1: \{-1,-2,-3\}', ...
       'Design 2: \{-0.5,-1,-2\}', ...
       'Design 3: \{-5,-6,-7\}', ...
       'Design 4: \{-2,-1\pm5i\}', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);
axis equal;

save_figure(fig2, 'fig/pole_placement_pole_map.png');

%% =====================================================================
%%  Figure 3: Effect of imaginary part (Design 1 vs Design 4)
%% =====================================================================
fig3 = figure('Units','inches','Position',[1 1 figsize.standard]);

subplot(2,1,1);
plot(t, Y1, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
plot(t, Y4, 'Color', colors.method2, 'LineWidth', lw.response);
yline(0, 'Color', colors.reference, 'LineWidth', 0.8, 'LineStyle', '--');
xlabel('Time [s]');
ylabel('Output y = x_1');
title('Effect of Imaginary-Part Pole Locations');
legend(sprintf('Design 1: poles = {%s}', num2str(p1, '%g, ')), ...
       'Design 4: poles = \{-2, -1\pm5i\}', ...
       'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

subplot(2,1,2);
plot(t, U1, 'Color', colors.method1, 'LineWidth', lw.response); hold on;
plot(t, U4, 'Color', colors.method2, 'LineWidth', lw.response);
xlabel('Time [s]');
ylabel('Control input u');
title('Control Input');
legend('Design 1', 'Design 4', 'Location', 'best');
grid on; set(gca, 'GridAlpha', 0.3);

save_figure(fig3, 'fig/pole_placement_imaginary_effect.png');

fprintf('\nAll figures saved to fig/ folder.\n');

%% =====================================================================
%%  Local function: state feedback simulation (4th-order Runge-Kutta)
%% =====================================================================
function [X, U] = sim_state_fb(A, B, K, x0, t)
    n = length(x0);
    Nt = length(t);
    dt = t(2) - t(1);
    X = zeros(n, Nt);
    U = zeros(1, Nt);
    X(:,1) = x0;
    U(1) = -K * x0;
    for k = 1:Nt-1
        xk = X(:,k);
        uk = -K * xk;
        U(k) = uk;
        % RK4
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
