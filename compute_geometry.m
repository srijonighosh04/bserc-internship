function geom = compute_geometry(params)
%COMPUTE_GEOMETRY Compute wing planform geometry, induced drag factor, and weight.
%
%   geom = compute_geometry(params) computes derived geometric and mass
%   properties for a cropped delta (trapezoidal) wing from the input
%   parameter struct.
%
%   Required fields in params:
%       Cr   - Root chord [m] (> 0)
%       Ct   - Tip chord [m] (>= 0)
%       b    - Wingspan [m] (> 0)
%       e    - Oswald span efficiency factor [-] (> 0)
%       mass - Aircraft mass [kg] (> 0)
%       g    - Gravitational acceleration [m/s^2] (> 0)
%
%   Returned struct geom fields:
%       lambda - Taper ratio (Ct / Cr) [-]
%       S      - Planform reference area [m^2]
%       AR     - Wing aspect ratio (b^2 / S) [-]
%       k      - Induced drag factor, 1 / (pi * e * AR) [-]
%       W      - Aircraft weight, mass * g [N]
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 1 || ~isstruct(params)
        error('compute_geometry:InvalidInput', 'Input "params" must be a struct.');
    end

    required_fields = {'Cr', 'Ct', 'b', 'e', 'mass', 'g'};
    for idx = 1:numel(required_fields)
        fname = required_fields{idx};
        if ~isfield(params, fname)
            error('compute_geometry:MissingField', ...
                  'Missing required field "%s" in params struct.', fname);
        end
    end

    if params.Cr <= 0 || params.Ct < 0 || params.b <= 0
        error('compute_geometry:InvalidGeometry', ...
              'Geometry requires Cr > 0, Ct >= 0, and b > 0.');
    end
    if params.e <= 0 || params.mass <= 0 || params.g <= 0
        error('compute_geometry:InvalidParameters', ...
              'Parameters e, mass, and g must all be strictly positive.');
    end

    geom = struct();
    geom.lambda = params.Ct / params.Cr;
    geom.S      = 0.5 * params.b * params.Cr * (1 + geom.lambda);
    geom.AR     = (params.b ^ 2) / geom.S;
    geom.k      = 1 / (pi * params.e * geom.AR);
    geom.W      = params.mass * params.g;
end
