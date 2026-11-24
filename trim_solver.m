function trim = trim_solver(alpha_deg, params)
%TRIM_SOLVER Solve longitudinal pitch trim (Cm = 0) across angle-of-attack sweep.
%
%   trim = trim_solver(alpha_deg, params) computes the elevator deflection
%   required for zero pitching moment (Cm = 0) and the resulting trimmed lift
%   coefficient at each angle of attack.
%
%   Inputs:
%       alpha_deg - Scalar or vector of angles of attack [deg]
%       params    - Struct containing aerodynamic derivatives:
%                     CL0     - Zero-AoA lift coefficient [-]
%                     CLalpha - Lift-curve slope [1/rad]
%                     CLde    - Lift due to elevator deflection [1/rad]
%                     Cm0     - Zero-AoA pitching moment coefficient [-]
%                     Cmalpha - Pitching moment slope [1/rad]
%                     Cmde    - Elevator control effectiveness [1/rad] (~= 0)
%
%   Returned struct trim fields (row vectors):
%       alpha_deg - Angle of attack [deg]
%       alpha_rad - Angle of attack [rad]
%       de_rad    - Trimmed elevator deflection [rad]
%       de_deg    - Trimmed elevator deflection [deg]
%       CL        - Trimmed lift coefficient [-]
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 2 || ~isstruct(params)
        error('trim_solver:InvalidInput', ...
              'Inputs must be (alpha_deg, params) where params is a struct.');
    end

    if ~isnumeric(alpha_deg) || isempty(alpha_deg)
        error('trim_solver:InvalidAlpha', ...
              'alpha_deg must be a non-empty numeric scalar or vector.');
    end

    required_fields = {'CL0', 'CLalpha', 'CLde', 'Cm0', 'Cmalpha', 'Cmde'};
    for idx = 1:numel(required_fields)
        fname = required_fields{idx};
        if ~isfield(params, fname)
            error('trim_solver:MissingField', ...
                  'Missing required field "%s" in params struct.', fname);
        end
    end

    if params.Cmde == 0
        error('trim_solver:ZeroControlPower', ...
              'params.Cmde cannot be zero (pitch trim is undefined).');
    end

    % Ensure row vector for consistent element-wise calculations
    alpha_deg_row = alpha_deg(:).';
    alpha_rad_row = alpha_deg_row * (pi / 180);

    % Enforce Cm = Cm0 + Cmalpha * alpha + Cmde * de = 0
    de_rad = -(params.Cm0 + params.Cmalpha .* alpha_rad_row) ./ params.Cmde;
    de_deg = de_rad * (180 / pi);

    % Trimmed lift coefficient at solved elevator deflection
    CL = params.CL0 + params.CLalpha .* alpha_rad_row + params.CLde .* de_rad;

    trim = struct();
    trim.alpha_deg = alpha_deg_row;
    trim.alpha_rad = alpha_rad_row;
    trim.de_rad    = de_rad;
    trim.de_deg    = de_deg;
    trim.CL        = CL;
end
