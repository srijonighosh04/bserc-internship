function perf = performance(trim, geom, params)
%PERFORMANCE Compute steady level flight thrust and power performance.
%
%   perf = performance(trim, geom, params) evaluates drag coefficient,
%   steady level-flight airspeed (L = W), thrust required (Tr = D), power
%   required (Pr = Tr * V), aerodynamic efficiency (CL / CD), and endurance
%   parameter (CL^(3/2) / CD).
%
%   Points with CL <= 0 cannot sustain steady level flight (L = W > 0) and
%   are assigned NaN for V, Tr, Pr, LD, and CL32_CD to prevent complex roots
%   or division by zero.
%
%   Inputs:
%       trim   - Struct returned by trim_solver
%       geom   - Struct returned by compute_geometry
%       params - Struct containing fields:
%                  CD0 - Zero-lift drag coefficient [-] (> 0)
%                  rho - Air density [kg/m^3] (> 0)
%
%   Returned struct perf fields:
%       alpha_deg  - Angle of attack [deg]
%       de_deg     - Trimmed elevator deflection [deg]
%       CL         - Trimmed lift coefficient [-]
%       CD         - Total drag coefficient [-]
%       V          - Level-flight airspeed [m/s]
%       Tr         - Thrust required [N]
%       Pr         - Power required [W]
%       LD         - Lift-to-drag ratio (CL / CD) [-]
%       CL32_CD    - Endurance parameter (CL^(3/2) / CD) [-]
%       valid_mask - Logical mask indicating CL > 0
%       summary    - Struct of extrema (min_Tr, min_Pr, max_LD, max_CL32_CD)
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 3 || ~isstruct(trim) || ~isstruct(geom) || ~isstruct(params)
        error('performance:InvalidInput', ...
              'Inputs must be structs: (trim, geom, params).');
    end

    if ~isfield(params, 'CD0') || ~isfield(params, 'rho')
        error('performance:MissingField', ...
              'params struct must contain fields "CD0" and "rho".');
    end
    if params.CD0 <= 0 || params.rho <= 0
        error('performance:InvalidParameters', ...
              'params.CD0 and params.rho must be strictly positive.');
    end

    CL = trim.CL(:).';
    n  = numel(CL);

    % Parabolic drag polar
    CD = params.CD0 + geom.k .* (CL .^ 2);

    % Preallocate with NaN for non-positive CL points
    V       = NaN(1, n);
    Tr      = NaN(1, n);
    Pr      = NaN(1, n);
    LD      = NaN(1, n);
    CL32_CD = NaN(1, n);

    valid = CL > 0;
    if ~all(valid)
        warning('performance:NonPositiveCL', ...
                '%d point(s) have CL <= 0; V, Tr, Pr, LD, CL32_CD set to NaN.', ...
                sum(~valid));
    end

    if any(valid)
        CL_v = CL(valid);
        CD_v = CD(valid);

        V_v  = sqrt((2 * geom.W) ./ (params.rho * geom.S .* CL_v));
        Tr_v = 0.5 * params.rho .* (V_v .^ 2) * geom.S .* CD_v;
        Pr_v = Tr_v .* V_v;
        LD_v = CL_v ./ CD_v;
        E_v  = (CL_v .^ 1.5) ./ CD_v;

        V(valid)       = V_v;
        Tr(valid)      = Tr_v;
        Pr(valid)      = Pr_v;
        LD(valid)      = LD_v;
        CL32_CD(valid) = E_v;
    end

    perf = struct();
    perf.alpha_deg  = trim.alpha_deg(:).';
    perf.de_deg     = trim.de_deg(:).';
    perf.CL         = CL;
    perf.CD         = CD;
    perf.V          = V;
    perf.Tr         = Tr;
    perf.Pr         = Pr;
    perf.LD         = LD;
    perf.CL32_CD    = CL32_CD;
    perf.valid_mask = valid;

    % Compute extrema over valid steady-level-flight points
    summary = struct();
    if any(valid)
        [min_Tr_val, iT]  = min(Tr);
        [min_Pr_val, iP]  = min(Pr);
        [max_LD_val, iLD] = max(LD);
        [max_E_val,  iE]  = max(CL32_CD);

        summary.min_Tr      = struct('value', min_Tr_val, 'alpha_deg', perf.alpha_deg(iT),  'V', V(iT));
        summary.min_Pr      = struct('value', min_Pr_val, 'alpha_deg', perf.alpha_deg(iP),  'V', V(iP));
        summary.max_LD      = struct('value', max_LD_val, 'alpha_deg', perf.alpha_deg(iLD), 'V', V(iLD));
        summary.max_CL32_CD = struct('value', max_E_val,  'alpha_deg', perf.alpha_deg(iE),  'V', V(iE));
    else
        empty_stat = struct('value', NaN, 'alpha_deg', NaN, 'V', NaN);
        summary.min_Tr      = empty_stat;
        summary.min_Pr      = empty_stat;
        summary.max_LD      = empty_stat;
        summary.max_CL32_CD = empty_stat;
    end
    perf.summary = summary;
end
