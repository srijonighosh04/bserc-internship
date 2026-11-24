function vtrim = velocity_trim_solver(V_target, geom, params)
%VELOCITY_TRIM_SOLVER Iteratively solve trim state for given flight velocities.
%
%   vtrim = velocity_trim_solver(V_target, geom, params) uses fzero to solve
%   steady level-flight lift equilibrium:
%       L(alpha) - W = 0.5 * rho * V^2 * S * CL_trim(alpha) - W = 0
%   at each requested airspeed V in V_target, where CL_trim(alpha) is
%   evaluated at zero pitching moment (Cm = 0) via trim_solver.
%
%   Inputs:
%       V_target - Scalar or vector of target airspeeds [m/s] (> 0)
%       geom     - Struct from compute_geometry
%       params   - Parameter struct
%
%   Returned struct vtrim fields (row vectors):
%       V_target         - Requested airspeeds [m/s]
%       alpha_deg        - Solved trim angles of attack [deg]
%       alpha_rad        - Solved trim angles of attack [rad]
%       de_rad           - Solved trim elevator deflections [rad]
%       de_deg           - Solved trim elevator deflections [deg]
%       CL               - Trimmed lift coefficients [-]
%       CD               - Total drag coefficients [-]
%       V                - Recomputed airspeeds [m/s] (matches V_target)
%       Tr               - Thrust required [N]
%       Pr               - Power required [W]
%       LD               - Lift-to-drag ratios [-]
%       CL32_CD          - Endurance parameters [-]
%       lift_residual    - Lift equilibrium residuals (L - W) [N]
%       max_abs_residual - Maximum |L - W| across all velocities [N]
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 3 || ~isstruct(geom) || ~isstruct(params)
        error('velocity_trim_solver:InvalidInput', ...
              'Inputs must be (V_target, geom, params).');
    end
    if ~isnumeric(V_target) || isempty(V_target) || any(V_target <= 0)
        error('velocity_trim_solver:InvalidVelocity', ...
              'V_target must be a non-empty numeric vector of positive speeds.');
    end

    V_row = V_target(:).';
    n = numel(V_row);
    alpha_sol_deg = NaN(1, n);
    opts = optimset('TolX', 1e-10);

    for idx = 1:n
        V_i = V_row(idx);
        res_fn = @(a_deg) lift_equilibrium_residual(a_deg, V_i, geom, params);

        % Bracket root across wide physical/linear alpha range
        bracket = [-45, 75];
        f_lo = res_fn(bracket(1));
        f_hi = res_fn(bracket(2));
        if f_lo * f_hi > 0
            bracket = [-85, 85];
            f_lo = res_fn(bracket(1));
            f_hi = res_fn(bracket(2));
        end

        if f_lo * f_hi <= 0
            alpha_sol_deg(idx) = fzero(res_fn, bracket, opts);
        else
            alpha_sol_deg(idx) = fzero(res_fn, 5.0, opts);
        end
    end

    % Evaluate consistent trim and performance states at solved alpha_deg
    trim = trim_solver(alpha_sol_deg, params);
    perf = performance(trim, geom, params);

    lift_at_sol   = 0.5 * params.rho .* (V_row .^ 2) * geom.S .* trim.CL;
    lift_residual = lift_at_sol - geom.W;

    vtrim = struct();
    vtrim.V_target         = V_row;
    vtrim.alpha_deg        = trim.alpha_deg;
    vtrim.alpha_rad        = trim.alpha_rad;
    vtrim.de_rad           = trim.de_rad;
    vtrim.de_deg           = trim.de_deg;
    vtrim.CL               = perf.CL;
    vtrim.CD               = perf.CD;
    vtrim.V                = perf.V;
    vtrim.Tr               = perf.Tr;
    vtrim.Pr               = perf.Pr;
    vtrim.LD               = perf.LD;
    vtrim.CL32_CD          = perf.CL32_CD;
    vtrim.lift_residual    = lift_residual;
    vtrim.max_abs_residual = max(abs(lift_residual));
end

function res = lift_equilibrium_residual(alpha_deg, V_i, geom, params)
%LIFT_EQUILIBRIUM_RESIDUAL Compute L(alpha) - W for fzero root-finding.
    t = trim_solver(alpha_deg, params);
    L = 0.5 * params.rho * (V_i ^ 2) * geom.S * t.CL;
    res = L - geom.W;
end
