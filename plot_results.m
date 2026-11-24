function fig = plot_results(perf, output_file, opts)
%PLOT_RESULTS Generate 3x3 trim and performance figure and optionally save PNG.
%
%   fig = plot_results(perf) creates a 3x3 subplot figure summarizing trim
%   and steady level-flight performance metrics.
%
%   fig = plot_results(perf, output_file) also saves the figure as a PNG
%   image using print('-dpng').
%
%   fig = plot_results(perf, output_file, opts) supports optional fields:
%       fig_name        - Figure window title string
%       mark_indicative - Logical; if true, marks alpha > stall_limit_deg
%                         as indicative (linear model has no stall) and
%                         highlights the minimum-power point
%       stall_limit_deg - Threshold angle of attack [deg] (default: 12)
%       vel_overlay     - Optional struct from velocity_trim_solver to
%                         overlay iterative V-sweep results on alpha-sweep
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 1 || ~isstruct(perf)
        error('plot_results:InvalidInput', 'Input "perf" must be a struct.');
    end
    if nargin < 2
        output_file = '';
    end
    if nargin < 3 || ~isstruct(opts)
        opts = struct();
    end

    fig_name        = get_opt(opts, 'fig_name', 'Trim & Performance');
    mark_indicative = get_opt(opts, 'mark_indicative', false);
    stall_limit_deg = get_opt(opts, 'stall_limit_deg', 12);
    vel_overlay     = get_opt(opts, 'vel_overlay', []);
    has_overlay     = isstruct(vel_overlay) && ~isempty(vel_overlay);

    % Find velocity corresponding to stall_limit_deg for V-axis markers
    V_limit = NaN;
    if mark_indicative
        [~, idx_lim] = min(abs(perf.alpha_deg - stall_limit_deg));
        if ~isempty(idx_lim) && perf.valid_mask(idx_lim)
            V_limit = perf.V(idx_lim);
        end
    end

    fig = figure('Name', fig_name, 'Position', [50, 50, 1200, 800]);

    % 1. Trim alpha vs V
    subplot(3, 3, 1);
    plot(perf.V, perf.alpha_deg, 'b-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.V, vel_overlay.alpha_deg, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_v_limit_marker(V_limit, stall_limit_deg);
    end
    grid on;
    xlabel('V (m/s)');
    ylabel('Trim \alpha (deg)');
    if mark_indicative
        title('Trim \alpha (no stall; >12\circ indicative)');
    else
        title('Trim angle of attack');
    end
    if has_overlay
        legend('Alpha sweep', 'Iterative V-solver', 'Location', 'northeast');
    end

    % 2. Trim elevator deflection vs alpha
    subplot(3, 3, 2);
    plot(perf.alpha_deg, perf.de_deg, 'r-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.alpha_deg, vel_overlay.de_deg, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_alpha_limit_marker(stall_limit_deg, 'Linear (no stall): >12\circ indicative');
    end
    grid on;
    xlabel('\alpha (deg)');
    ylabel('\delta_{e,trim} (deg)');
    title('Trim elevator deflection');

    % 3. Lift coefficient vs alpha
    subplot(3, 3, 3);
    plot(perf.alpha_deg, perf.CL, 'k-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.alpha_deg, vel_overlay.CL, 'rx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_alpha_limit_marker(stall_limit_deg, 'No stall modeled (>12\circ indicative)');
    end
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L (-)');
    title('Lift coefficient');

    % 4. Drag coefficient vs alpha
    subplot(3, 3, 4);
    plot(perf.alpha_deg, perf.CD, 'm-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.alpha_deg, vel_overlay.CD, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_alpha_limit_marker(stall_limit_deg, '>12\circ indicative');
    end
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_D (-)');
    title('Drag coefficient');

    % 5. Thrust required vs V
    subplot(3, 3, 5);
    plot(perf.V, perf.Tr, 'b-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.V, vel_overlay.Tr, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_v_limit_marker(V_limit, stall_limit_deg);
    end
    grid on;
    xlabel('V (m/s)');
    ylabel('T_r (N)');
    title('Thrust required');

    % 6. Power required vs V
    subplot(3, 3, 6);
    plot(perf.V, perf.Pr, 'r-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.V, vel_overlay.Pr, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_v_limit_marker(V_limit, stall_limit_deg);
        [~, iP] = min(perf.Pr);
        if ~isempty(iP) && isfinite(perf.Pr(iP))
            plot(perf.V(iP), perf.Pr(iP), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            text(perf.V(iP) + 1.5, perf.Pr(iP), ...
                 sprintf('Min Pr=%.1f W (C_L=%.2f)', perf.Pr(iP), perf.CL(iP)), ...
                 'FontSize', 7, 'Color', [0, 0.45, 0], 'VerticalAlignment', 'bottom');
        end
    end
    grid on;
    xlabel('V (m/s)');
    ylabel('P_r (W)');
    title('Power required');

    % 7. Lift-to-drag ratio vs alpha
    subplot(3, 3, 7);
    plot(perf.alpha_deg, perf.LD, 'g-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.alpha_deg, vel_overlay.LD, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_alpha_limit_marker(stall_limit_deg, '>12\circ indicative');
    end
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L/C_D (-)');
    title('Lift-to-drag ratio');

    % 8. Endurance parameter vs alpha
    subplot(3, 3, 8);
    plot(perf.alpha_deg, perf.CL32_CD, 'c-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.alpha_deg, vel_overlay.CL32_CD, 'kx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_alpha_limit_marker(stall_limit_deg, '>12\circ indicative');
        [~, iE] = max(perf.CL32_CD);
        if ~isempty(iE) && isfinite(perf.CL32_CD(iE))
            plot(perf.alpha_deg(iE), perf.CL32_CD(iE), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            text(perf.alpha_deg(iE) - 6.5, perf.CL32_CD(iE) * 0.90, ...
                 sprintf('Max (C_L=%.2f, \\alpha=%.1f\\circ)', perf.CL(iE), perf.alpha_deg(iE)), ...
                 'FontSize', 7, 'Color', [0, 0.45, 0]);
        end
    end
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L^{3/2}/C_D (-)');
    title('Endurance parameter (min power)');

    % 9. CL vs V
    subplot(3, 3, 9);
    plot(perf.V, perf.CL, 'k-o', 'MarkerSize', 3);
    hold on;
    if has_overlay
        plot(vel_overlay.V, vel_overlay.CL, 'rx', 'MarkerSize', 5);
    end
    if mark_indicative
        add_v_limit_marker(V_limit, stall_limit_deg);
    end
    grid on;
    xlabel('V (m/s)');
    ylabel('C_L (-)');
    title('C_L vs velocity');

    if ~isempty(output_file)
        print(fig, output_file, '-dpng', '-r150');
    end
end

function val = get_opt(opts, field, default_val)
%GET_OPT Safely extract optional struct field with default fallback.
    if isfield(opts, field)
        val = opts.(field);
    else
        val = default_val;
    end
end

function add_alpha_limit_marker(limit_deg, label_str)
%ADD_ALPHA_LIMIT_MARKER Draw dashed threshold line at alpha = limit_deg.
    yl = ylim;
    plot([limit_deg, limit_deg], yl, 'r--', 'LineWidth', 1.1);
    ylim(yl);
    y_pos = yl(1) + 0.12 * (yl(2) - yl(1));
    text(limit_deg + 0.3, y_pos, label_str, ...
         'Color', 'r', 'FontSize', 7, 'VerticalAlignment', 'bottom');
end

function add_v_limit_marker(V_limit, limit_deg)
%ADD_V_LIMIT_MARKER Draw dashed threshold line at V corresponding to limit_deg.
    if isnan(V_limit)
        return;
    end
    yl = ylim;
    plot([V_limit, V_limit], yl, 'r--', 'LineWidth', 1.1);
    ylim(yl);
    y_pos = yl(1) + 0.85 * (yl(2) - yl(1));
    text(V_limit + 1.0, y_pos, ...
         sprintf('V(\\alpha=%.0f\\circ)=%.1f m/s\n(left: >%.0f\\circ indicative)', ...
                 limit_deg, V_limit, limit_deg), ...
         'Color', 'r', 'FontSize', 6.5, 'VerticalAlignment', 'top');
end
