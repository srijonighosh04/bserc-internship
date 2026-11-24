function varargout = validate_model()
%VALIDATE_MODEL Verify numerical UAV trim/performance results against theory.
%
%   validate_model() runs:
%     1) Assert-based unit tests for geometry (S, AR, k), pitch trim
%        equilibrium (Cm == 0 at every trimmed point), and iterative
%        velocity-based trim solver consistency with the alpha sweep.
%     2) Exact analytical check of trim_solver and performance at the
%        closed-form optimal lift coefficients.
%     3) Pass/fail comparison tables with percentage errors comparing
%        closed-form parabolic drag-polar optima against:
%          - Primary sweep:  alpha = 0:0.5:12 deg
%          - Extended sweep: alpha = -2:0.5:20 deg (shows min-power point)
%
%   report = validate_model() optionally returns a struct containing the
%   theoretical values, sweep values, percentage errors, and pass counts.
%
%   Compatible with MATLAB and GNU Octave.

    %% 1. MODEL PARAMETERS (MATCHING main_run.m) -----------------------------
    params = struct();
    params.Cr      = 0.9;
    params.Ct      = 0.15;
    params.b       = 1.5;
    params.e       = 0.89;
    params.CD0     = 0.03;
    params.Cm0     = 0.01;
    params.CL0     = 0.0;
    params.CLalpha = 2.92;
    params.CLde    = 0.265;
    params.Cmalpha = -0.292;
    params.Cmde    = -0.4;
    params.mass    = 3.5;
    params.rho     = 1.225;
    params.g       = 10.0;

    alpha_nom_deg = 0:0.5:12;
    alpha_ext_deg = -2:0.5:20;
    V_sweep_mps   = 8:0.5:30;

    %% 2. RUN NUMERICAL PIPELINE ------------------------------------------
    geom     = compute_geometry(params);
    trim_nom = trim_solver(alpha_nom_deg, params);
    perf_nom = performance(trim_nom, geom, params);
    trim_ext = trim_solver(alpha_ext_deg, params);
    perf_ext = performance(trim_ext, geom, params);
    vtrim    = velocity_trim_solver(V_sweep_mps, geom, params);

    %% 3. ASSERT-BASED UNIT TESTS -----------------------------------------
    fprintf('=== 1. Assert-Based Unit Tests ===\n');

    % 3a. Geometry unit tests (expected: S = 0.7875 m^2, AR = 2.857, k = 0.1252)
    assert(abs(geom.S - 0.7875) < 1e-4, ...
           'Geometry test failed: expected S = 0.7875 m^2, got %.6f', geom.S);
    assert(abs(geom.AR - 2.857) < 1e-3, ...
           'Geometry test failed: expected AR = 2.857, got %.6f', geom.AR);
    assert(abs(geom.k - 0.1252) < 1e-4, ...
           'Geometry test failed: expected k = 0.1252, got %.6f', geom.k);
    assert(abs(geom.lambda - (1 / 6)) < 1e-12, ...
           'Geometry test failed: expected lambda = 1/6, got %.6f', geom.lambda);
    assert(abs(geom.W - 35.0) < 1e-12, ...
           'Geometry test failed: expected W = 35.0 N, got %.6f', geom.W);
    fprintf('[PASS] Geometry assertions: S = %.4f m^2, AR = %.4f, k = %.4f, W = %.2f N\n', ...
            geom.S, geom.AR, geom.k, geom.W);

    % 3b. Pitch trim equilibrium unit tests (Cm must be ~0 at every point)
    Cm_nom = params.Cm0 + params.Cmalpha .* trim_nom.alpha_rad + ...
             params.Cmde .* trim_nom.de_rad;
    max_Cm_nom = max(abs(Cm_nom));
    assert(max_Cm_nom < 1e-12, ...
           'Trim test failed (primary): max |Cm| = %.3e >= 1e-12', max_Cm_nom);

    Cm_ext = params.Cm0 + params.Cmalpha .* trim_ext.alpha_rad + ...
             params.Cmde .* trim_ext.de_rad;
    max_Cm_ext = max(abs(Cm_ext));
    assert(max_Cm_ext < 1e-12, ...
           'Trim test failed (extended): max |Cm| = %.3e >= 1e-12', max_Cm_ext);
    fprintf('[PASS] Trim Cm == 0 assertions: max |Cm| = %.2e (primary), %.2e (extended)\n', ...
            max_Cm_nom, max_Cm_ext);

    % 3c. Iterative velocity-solver unit tests (L == W and alpha-sweep match)
    assert(vtrim.max_abs_residual < 1e-8, ...
           'Velocity solver L == W residual too large: %.3e N', vtrim.max_abs_residual);
    vcheck_nom = velocity_trim_solver(perf_nom.V, geom, params);
    max_da = max(abs(vcheck_nom.alpha_deg - perf_nom.alpha_deg));
    max_de = max(abs(vcheck_nom.de_deg    - perf_nom.de_deg));
    max_dT = max(abs(vcheck_nom.Tr        - perf_nom.Tr));
    max_dP = max(abs(vcheck_nom.Pr        - perf_nom.Pr));
    assert(max_da < 1e-8 && max_de < 1e-8 && max_dT < 1e-8 && max_dP < 1e-7, ...
           'Iterative velocity solver did not match alpha sweep within tolerance.');
    fprintf('[PASS] Iterative V-solver assertions: max |L-W| = %.2e N, max |d_alpha| = %.2e deg\n', ...
            vtrim.max_abs_residual, max_da);

    %% 4. CLOSED-FORM PARABOLIC DRAG POLAR THEORY -------------------------
    theory = compute_closed_form_theory(params, geom);

    % Exact analytical verification of trim_solver + performance
    trim_exact = trim_solver([theory.max_LD.alpha_deg, theory.max_E.alpha_deg], params);
    perf_exact = performance(trim_exact, geom, params);
    assert(abs(perf_exact.LD(1) - theory.max_LD.LD) < 1e-12, ...
           'Exact (L/D)max mismatch against theory.');
    assert(abs(perf_exact.Tr(1) - theory.max_LD.Tr) < 1e-12, ...
           'Exact Tr_min mismatch against theory.');
    assert(abs(perf_exact.CL32_CD(2) - theory.max_E.CL32_CD) < 1e-12, ...
           'Exact (CL^1.5/CD)max mismatch against theory.');
    assert(abs(perf_exact.Pr(2) - theory.max_E.Pr) < 1e-12, ...
           'Exact Pr_min mismatch against theory.');
    fprintf('[PASS] Exact analytical solver check: all errors < 1e-12\n\n');

    %% 5. COMPARISON TABLE: CLOSED-FORM vs PRIMARY SWEEP (0:0.5:12 deg) ---
    tol_nom_pct = 2.5;
    fprintf('=== 2. Closed-Form Theory vs. Primary Sweep (alpha = 0:0.5:12 deg, tol = %.1f%%) ===\n', ...
            tol_nom_pct);
    fprintf('Note: Theoretical max CL^1.5/CD (min Pr) occurs at alpha = %.2f deg (> 12.0 deg limit).\n', ...
            theory.max_E.alpha_deg);
    res_nom = print_comparison_table(theory, perf_nom, tol_nom_pct);

    %% 6. COMPARISON TABLE: CLOSED-FORM vs EXTENDED SWEEP (-2:0.5:20 deg) -
    tol_ext_pct = 2.5;
    fprintf('\n=== 3. Closed-Form Theory vs. Extended Sweep (alpha = -2:0.5:20 deg, tol = %.1f%%) ===\n', ...
            tol_ext_pct);
    fprintf('Note: Extended sweep captures min Pr near CL ~ 0.85 (no stall; >12 deg indicative).\n');
    res_ext = print_comparison_table(theory, perf_ext, tol_ext_pct);

    assert(res_ext.num_failed == 0, ...
           'Extended sweep (-2:0.5:20 deg) failed %d theoretical comparison(s) at %.1f%% tolerance.', ...
           res_ext.num_failed, tol_ext_pct);
    fprintf('\n[PASS] Extended sweep (-2:0.5:20 deg) matched both closed-form optima within %.1f%%.\n', ...
            tol_ext_pct);

    if nargout > 0
        report = struct();
        report.geom     = geom;
        report.theory   = theory;
        report.nominal  = res_nom;
        report.extended = res_ext;
        report.vtrim    = vtrim;
        report.max_Cm   = max_Cm_nom;
        varargout{1}    = report;
    end
end

%% ========================================================================
%  LOCAL HELPER: CLOSED-FORM THEORY
%  ========================================================================
function theory = compute_closed_form_theory(params, geom)
%COMPUTE_CLOSED_FORM_THEORY Analytical optima for CD = CD0 + k*CL^2.

    % Effective trimmed lift curve: CL(alpha) = CL_t0 + CLa_t * alpha_rad
    CL_t0 = params.CL0 - (params.CLde * params.Cm0) / params.Cmde;
    CLa_t = params.CLalpha - (params.CLde * params.Cmalpha) / params.Cmde;

    % --- Point 1: Max L/D (also Minimum Thrust Required) -----------------
    CL1     = sqrt(params.CD0 / geom.k);
    CD1     = 2 * params.CD0;
    LDmax   = 1 / (2 * sqrt(geom.k * params.CD0));
    Tr_min  = geom.W / LDmax;
    a1_rad  = (CL1 - CL_t0) / CLa_t;
    de1_rad = -(params.Cm0 + params.Cmalpha * a1_rad) / params.Cmde;
    V1      = sqrt((2 * geom.W) / (params.rho * geom.S * CL1));
    Pr1     = Tr_min * V1;
    E1      = (CL1 ^ 1.5) / CD1;

    p1 = struct();
    p1.CL        = CL1;
    p1.CD        = CD1;
    p1.LD        = LDmax;
    p1.CL32_CD   = E1;
    p1.alpha_deg = a1_rad * (180 / pi);
    p1.de_deg    = de1_rad * (180 / pi);
    p1.V         = V1;
    p1.Tr        = Tr_min;
    p1.Pr        = Pr1;

    % --- Point 2: Max CL^1.5/CD (also Minimum Power Required) ------------
    CL2     = sqrt((3 * params.CD0) / geom.k);
    CD2     = 4 * params.CD0;
    LD2     = CL2 / CD2;
    Emax    = (CL2 ^ 1.5) / CD2;
    Tr2     = geom.W / LD2;
    a2_rad  = (CL2 - CL_t0) / CLa_t;
    de2_rad = -(params.Cm0 + params.Cmalpha * a2_rad) / params.Cmde;
    V2      = sqrt((2 * geom.W) / (params.rho * geom.S * CL2));
    Pr_min  = Tr2 * V2;

    p2 = struct();
    p2.CL        = CL2;
    p2.CD        = CD2;
    p2.LD        = LD2;
    p2.CL32_CD   = Emax;
    p2.alpha_deg = a2_rad * (180 / pi);
    p2.de_deg    = de2_rad * (180 / pi);
    p2.V         = V2;
    p2.Tr        = Tr2;
    p2.Pr        = Pr_min;

    theory = struct();
    theory.max_LD = p1;
    theory.max_E  = p2;
end

%% ========================================================================
%  LOCAL HELPER: PRINT PASS/FAIL COMPARISON TABLE
%  ========================================================================
function res = print_comparison_table(theory, perf, tol_pct)
%PRINT_COMPARISON_TABLE Compare sweep extrema to theory and print table.

    [~, iLD] = max(perf.LD);
    [~, iE]  = max(perf.CL32_CD);

    % Build comparison rows: {Regime, Metric, TheoryVal, SweepVal}
    rows = { ...
        'Max L/D (Min Tr)',      'CL (-)',           theory.max_LD.CL,        perf.CL(iLD); ...
        'Max L/D (Min Tr)',      '(L/D)max (-)',     theory.max_LD.LD,        perf.LD(iLD); ...
        'Max L/D (Min Tr)',      'Tr_min (N)',       theory.max_LD.Tr,        perf.Tr(iLD); ...
        'Max L/D (Min Tr)',      'alpha (deg)',      theory.max_LD.alpha_deg, perf.alpha_deg(iLD); ...
        'Max L/D (Min Tr)',      'delta_e (deg)',    theory.max_LD.de_deg,    perf.de_deg(iLD); ...
        'Max L/D (Min Tr)',      'V (m/s)',          theory.max_LD.V,         perf.V(iLD); ...
        'Max L/D (Min Tr)',      'Pr (W)',           theory.max_LD.Pr,        perf.Pr(iLD); ...
        'Max CL^1.5/CD (Min Pr)','CL (-)',           theory.max_E.CL,         perf.CL(iE); ...
        'Max CL^1.5/CD (Min Pr)','(CL^1.5/CD)max',   theory.max_E.CL32_CD,    perf.CL32_CD(iE); ...
        'Max CL^1.5/CD (Min Pr)','Pr_min (W)',       theory.max_E.Pr,         perf.Pr(iE); ...
        'Max CL^1.5/CD (Min Pr)','alpha (deg)',      theory.max_E.alpha_deg,  perf.alpha_deg(iE); ...
        'Max CL^1.5/CD (Min Pr)','delta_e (deg)',    theory.max_E.de_deg,     perf.de_deg(iE); ...
        'Max CL^1.5/CD (Min Pr)','V (m/s)',          theory.max_E.V,          perf.V(iE); ...
        'Max CL^1.5/CD (Min Pr)','Tr (N)',           theory.max_E.Tr,         perf.Tr(iE) ...
    };

    n_rows = size(rows, 1);
    pct_errs = zeros(n_rows, 1);
    passed   = false(n_rows, 1);

    fprintf('%-23s %-15s %12s %12s %11s %-6s\n', ...
            'Operating Point', 'Quantity', 'Theory', 'Sweep', 'Err (%)', 'Status');
    fprintf('%s\n', repmat('-', 1, 84));

    for idx = 1:n_rows
        regime  = rows{idx, 1};
        metric  = rows{idx, 2};
        val_th  = rows{idx, 3};
        val_sw  = rows{idx, 4};
        pct_err = 100 * abs(val_sw - val_th) / abs(val_th);
        is_ok   = pct_err <= tol_pct;

        pct_errs(idx) = pct_err;
        passed(idx)   = is_ok;

        if is_ok
            status_str = 'PASS';
        else
            status_str = 'FAIL';
        end

        fprintf('%-23s %-15s %12.4f %12.4f %10.4f%% %-6s\n', ...
                regime, metric, val_th, val_sw, pct_err, status_str);
    end

    res = struct();
    res.pct_errs   = pct_errs;
    res.passed     = passed;
    res.num_passed = sum(passed);
    res.num_failed = sum(~passed);
    fprintf('Summary: %d/%d passed (tol = %.2f%%)\n', ...
            res.num_passed, n_rows, tol_pct);
end
