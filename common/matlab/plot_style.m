
%% plot_style.m  —  Common plot style for control_state_feedback repository
%
%  Usage:  Run  plot_style  at the beginning of each script, or call
%          the helper functions defined here.
%
%  Reference: Project instruction document (plot style specification).

%% --- Color definitions (unified across all clusters) ---
colors.reference = [0.000 0.000 0.000];   % #000000  black
colors.method1   = [0.122 0.467 0.706];   % #1f77b4  blue
colors.method2   = [0.839 0.153 0.157];   % #d62728  red
colors.method3   = [0.173 0.627 0.173];   % #2ca02c  green
colors.method4   = [1.000 0.498 0.055];   % #ff7f0e  orange
colors.control   = [0.580 0.404 0.741];   % #9467bd  purple
colors.noise     = [0.498 0.498 0.498];   % #7f7f7f  gray
colors.baseline  = [0.498 0.498 0.498];   % #7f7f7f  gray

%% --- Figure sizes (inches) ---
figsize.standard = [8, 5];    % 1200 x 750 px at 150 dpi
figsize.wide     = [8, 6];    % 1200 x 900 px
figsize.square   = [6, 6];    %  900 x 900 px

%% --- Default font settings ---
set(0, 'DefaultAxesFontSize', 13);
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 13);
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultLegendFontSize', 12);

%% --- Line width defaults ---
lw.reference = 2.0;
lw.response  = 1.5;
lw.baseline  = 1.5;
lw.noise     = 1.0;

%% --- Helper: create figure with standard size ---
% Usage:  fig = new_figure(figsize.standard);
%         fig = new_figure(figsize.square);

%% --- Helper: save figure as PNG ---
% Usage:  save_figure(fig, 'fig/pole_placement_step_response.png');
function save_figure(fig, filepath)
    exportgraphics(fig, filepath, 'Resolution', 150, ...
                   'BackgroundColor', 'white');
    fprintf('Saved: %s\n', filepath);
end
