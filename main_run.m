%% ========================================================================
%  Main Driver: Cropped Delta Wing UAV Trim & Performance Analysis
%  Steady Level Flight (Wing-Alone)
%
%  Cases:
%    Primary   : alpha = 0:0.5:12 deg  -> uav_trim_results.csv, uav_trim_plots.png
%    Extended  : alpha = -2:0.5:20 deg -> shows min-power point (CL ~ 0.85);
%                linear model has no stall (>12 deg indicative only)
%    Iterative : V = 8:0.5:30 m/s solved via fzero on L = W and compared to
%                alpha-sweep results
%
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

%% 2. PRIMARY DELIVERABLE: ALPHA = 0:0.5:12 DEG ---------------------------
alpha_deg = 0:0.5:12;

geom = compute_geometry(params);
trim = trim_solver(alpha_deg, params);
perf = performance(trim, geom, params);

fprintf('=== Cropped Delta Wing UAV: Geometry & Mass Summary ===\n');
fprintf('lambda = %.4f | S = %.4f m^2 | AR = %.4f | k = %.4f | W = %.2f N\n\n', ...
        geom.lambda, geom.S, geom.AR, geom.k, geom.W);

fprintf('=== Primary Case: alpha = 0:0.5:12 deg ===\n');
print_perf_table(perf);
write_perf_csv(perf, 'uav_trim_results.csv');
print_extrema_summary(perf, alpha_deg(1), alpha_deg(end));

% Primary deliverable plots (summary panel + 8 separate 300-dpi PNGs in figures/)
plot_results(perf, 'uav_trim_plots.png');
export_publication_figures(perf, 'figures');

%% 3. SECOND CASE: EXTENDED ALPHA SWEEP (-2:0.5:20 DEG) -------------------
alpha_ext_deg = -2:0.5:20;
trim_ext = trim_solver(alpha_ext_deg, params);
perf_ext = performance(trim_ext, geom, params);

fprintf('\n=== Extended Case: alpha = -2:0.5:20 deg (No Stall; >12 deg Indicative Only) ===\n');
fprintf('Note: Points with CL <= 0 (alpha <= -0.5 deg) cannot sustain L = W > 0 (NaN).\n');
print_extrema_summary(perf_ext, alpha_ext_deg(1), alpha_ext_deg(end));
[~, idx_minP_ext] = min(perf_ext.Pr);
fprintf('At min power: CL = %.4f (near theoretical sqrt(3*CD0/k) ~ 0.85)\n', ...
        perf_ext.CL(idx_minP_ext));
write_perf_csv(perf_ext, 'uav_trim_results_extended.csv');

%% 4. ITERATIVE VELOCITY SOLVER: V = 8:0.5:30 M/S -------------------------
V_sweep = 8:0.5:30;
vtrim   = velocity_trim_solver(V_sweep, geom, params);

fprintf('\n=== Iterative Velocity Solver (fzero on L = W): V = 8:0.5:30 m/s ===\n');
fprintf('Max |L - W| residual across V sweep: %.3e N\n', vtrim.max_abs_residual);
fprintf('%-8s %-10s %-12s %-9s %-9s %-9s %-10s %-11s\n', ...
        'V_mps', 'alpha_deg', 'delta_e_deg', 'CL', 'CD', 'Tr_N', 'Pr_W', 'L_minus_W');
fprintf('%s\n', repmat('-', 1, 86));
for idx = 1:numel(vtrim.V_target)
    fprintf('%8.2f %10.4f %12.4f %9.5f %9.5f %9.4f %10.3f %11.3e\n', ...
            vtrim.V_target(idx), vtrim.alpha_deg(idx), vtrim.de_deg(idx), ...
            vtrim.CL(idx), vtrim.CD(idx), vtrim.Tr(idx), vtrim.Pr(idx), ...
            vtrim.lift_residual(idx));
end
write_perf_csv(vtrim, 'uav_velocity_trim_results.csv');

% Cross-check: invert primary alpha-sweep velocities via iterative solver
vcheck_nom = velocity_trim_solver(perf.V, geom, params);
fprintf('\n=== Cross-Check: Iterative V-Solver vs Primary Alpha Sweep (0..12 deg) ===\n');
fprintf('Max |alpha_iter - alpha_sweep| : %.3e deg\n', max(abs(vcheck_nom.alpha_deg - perf.alpha_deg)));
fprintf('Max |de_iter    - de_sweep|    : %.3e deg\n', max(abs(vcheck_nom.de_deg    - perf.de_deg)));
fprintf('Max |Tr_iter    - Tr_sweep|    : %.3e N\n',   max(abs(vcheck_nom.Tr        - perf.Tr)));
fprintf('Max |Pr_iter    - Pr_sweep|    : %.3e W\n',   max(abs(vcheck_nom.Pr        - perf.Pr)));

% Extended plot with no-stall / >12 deg indicative markers + iterative V overlay
ext_opts = struct();
ext_opts.fig_name        = 'Extended Trim & Performance (-2 to 20 deg, Linear No-Stall)';
ext_opts.mark_indicative = true;
ext_opts.stall_limit_deg = 12;
ext_opts.vel_overlay     = vtrim;
plot_results(perf_ext, 'uav_trim_plots_extended.png', ext_opts);

%% ========================================================================
%  LOCAL HELPERS (COMPATIBLE WITH MATLAB R2016b+ & OCTAVE 6+)
%  ========================================================================
function print_perf_table(p)
    fprintf('%-9s %-12s %-9s %-9s %-10s %-9s %-10s %-9s %-10s\n', ...
            'alpha_deg', 'delta_e_deg', 'CL', 'CD', 'V_mps', ...
            'Tr_N', 'Pr_W', 'CL_CD', 'CL32_CD');
    fprintf('%s\n', repmat('-', 1, 93));
    for idx = 1:numel(p.alpha_deg)
        fprintf('%9.2f %12.4f %9.5f %9.5f %10.3f %9.4f %10.3f %9.4f %10.4f\n', ...
                p.alpha_deg(idx), p.de_deg(idx), p.CL(idx), p.CD(idx), ...
                p.V(idx), p.Tr(idx), p.Pr(idx), p.LD(idx), p.CL32_CD(idx));
    end
end

function print_extrema_summary(p, a_min, a_max)
    s = p.summary;
    fprintf('\nPerformance Extrema (over alpha = %.1f to %.1f deg):\n', a_min, a_max);
    fprintf('  Min thrust required : %.3f N  at alpha = %.1f deg, V = %.2f m/s\n', ...
            s.min_Tr.value, s.min_Tr.alpha_deg, s.min_Tr.V);
    fprintf('  Min power required  : %.3f W  at alpha = %.1f deg, V = %.2f m/s\n', ...
            s.min_Pr.value, s.min_Pr.alpha_deg, s.min_Pr.V);
    fprintf('  Max CL/CD           : %.3f   at alpha = %.1f deg\n', ...
            s.max_LD.value, s.max_LD.alpha_deg);
    fprintf('  Max CL^1.5/CD       : %.3f   at alpha = %.1f deg\n', ...
            s.max_CL32_CD.value, s.max_CL32_CD.alpha_deg);
end

function write_perf_csv(p, csv_file)
    fid = fopen(csv_file, 'w');
    if fid == -1
        error('main_run:FileOpenFailed', 'Could not open %s for writing.', csv_file);
    end
    fprintf(fid, 'alpha_deg,delta_e_deg,CL,CD,V_mps,Tr_N,Pr_W,CL_CD,CL32_CD\n');
    for idx = 1:numel(p.alpha_deg)
        fprintf(fid, '%.4f,%.6f,%.6f,%.6f,%.4f,%.6f,%.4f,%.6f,%.6f\n', ...
                p.alpha_deg(idx), p.de_deg(idx), p.CL(idx), p.CD(idx), ...
                p.V(idx), p.Tr(idx), p.Pr(idx), p.LD(idx), p.CL32_CD(idx));
    end
    fclose(fid);
end
