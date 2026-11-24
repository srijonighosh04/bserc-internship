%% ========================================================================
%  Main Driver: Cropped Delta Wing UAV Trim & Performance Analysis
%  Steady Level Flight (Wing-Alone)
%  Compatible with MATLAB and GNU Octave
%  ========================================================================
clear; clc; close all;

%% 1. INPUT PARAMETERS STRUCT ---------------------------------------------
params = struct();

% Wing geometry and span efficiency
params.Cr = 0.9;      % Root chord [m]
params.Ct = 0.15;     % Tip chord [m]
params.b  = 1.5;      % Wingspan [m]
params.e  = 0.89;     % Oswald span efficiency factor [-]

% Aerodynamic coefficients and stability/control derivatives
params.CD0     = 0.03;    % Zero-lift drag coefficient [-]
params.Cm0     = 0.01;    % Zero-alpha pitching moment coefficient [-]
params.CL0     = 0.0;     % Zero-alpha lift coefficient [-] (assumed 0; not given)
params.CLalpha = 2.92;    % Lift-curve slope [1/rad]
params.CLde    = 0.265;   % Lift due to elevator [1/rad]
params.Cmalpha = -0.292;  % Pitching moment slope [1/rad]
params.Cmde    = -0.4;    % Elevator control power [1/rad]

% Mass and atmospheric conditions
params.mass = 3.5;    % Aircraft mass [kg]
params.rho  = 1.225;  % Air density [kg/m^3]
params.g    = 10.0;   % Gravitational acceleration [m/s^2]

% Angle-of-attack sweep [deg]
alpha_deg = 0:0.5:12;

%% 2. COMPUTE GEOMETRY, TRIM, AND PERFORMANCE -----------------------------
geom = compute_geometry(params);
trim = trim_solver(alpha_deg, params);
perf = performance(trim, geom, params);

%% 3. DISPLAY GEOMETRY AND RESULTS TABLE ----------------------------------
fprintf('=== Cropped Delta Wing UAV: Geometry & Mass Summary ===\n');
fprintf('lambda = %.4f | S = %.4f m^2 | AR = %.4f | k = %.4f | W = %.2f N\n\n', ...
        geom.lambda, geom.S, geom.AR, geom.k, geom.W);

fprintf('%-9s %-12s %-9s %-9s %-10s %-9s %-10s %-9s %-10s\n', ...
        'alpha_deg', 'delta_e_deg', 'CL', 'CD', 'V_mps', ...
        'Tr_N', 'Pr_W', 'CL_CD', 'CL32_CD');
fprintf('%s\n', repmat('-', 1, 93));

for idx = 1:numel(perf.alpha_deg)
    fprintf('%9.2f %12.4f %9.5f %9.5f %10.3f %9.4f %10.3f %9.4f %10.4f\n', ...
            perf.alpha_deg(idx), perf.de_deg(idx), perf.CL(idx), perf.CD(idx), ...
            perf.V(idx), perf.Tr(idx), perf.Pr(idx), perf.LD(idx), perf.CL32_CD(idx));
end

%% 4. EXPORT CSV (PORTABLE ACROSS MATLAB & OCTAVE) ------------------------
csv_file = 'uav_trim_results.csv';
fid = fopen(csv_file, 'w');
if fid == -1
    error('main_run:FileOpenFailed', 'Could not open %s for writing.', csv_file);
end
fprintf(fid, 'alpha_deg,delta_e_deg,CL,CD,V_mps,Tr_N,Pr_W,CL_CD,CL32_CD\n');
for idx = 1:numel(perf.alpha_deg)
    fprintf(fid, '%.4f,%.6f,%.6f,%.6f,%.4f,%.6f,%.4f,%.6f,%.6f\n', ...
            perf.alpha_deg(idx), perf.de_deg(idx), perf.CL(idx), perf.CD(idx), ...
            perf.V(idx), perf.Tr(idx), perf.Pr(idx), perf.LD(idx), perf.CL32_CD(idx));
end
fclose(fid);

%% 5. DISPLAY PERFORMANCE EXTREMA -----------------------------------------
s = perf.summary;
fprintf('\n=== Performance Extrema (over alpha = %.1f to %.1f deg) ===\n', ...
        alpha_deg(1), alpha_deg(end));
fprintf('Min thrust required : %.3f N  at alpha = %.1f deg, V = %.2f m/s\n', ...
        s.min_Tr.value, s.min_Tr.alpha_deg, s.min_Tr.V);
fprintf('Min power required  : %.3f W  at alpha = %.1f deg, V = %.2f m/s\n', ...
        s.min_Pr.value, s.min_Pr.alpha_deg, s.min_Pr.V);
fprintf('Max CL/CD           : %.3f   at alpha = %.1f deg\n', ...
        s.max_LD.value, s.max_LD.alpha_deg);
fprintf('Max CL^1.5/CD       : %.3f   at alpha = %.1f deg\n', ...
        s.max_CL32_CD.value, s.max_CL32_CD.alpha_deg);

%% 6. GENERATE AND SAVE PLOTS ---------------------------------------------
plot_results(perf, 'uav_trim_plots.png');
