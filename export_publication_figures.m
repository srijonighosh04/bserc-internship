function export_publication_figures(perf, out_dir)
%EXPORT_PUBLICATION_FIGURES Save 8 separate 300-dpi publication PNG plots.
%
%   export_publication_figures(perf) saves 8 individual figures into the
%   'figures' folder at 300 dpi with axis labels (including units), titles,
%   grid, consistent styling, and annotated markers for minimum thrust,
%   minimum power, and maximum L/D.
%
%   export_publication_figures(perf, out_dir) specifies the output folder.
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 1 || ~isstruct(perf)
        error('export_publication_figures:InvalidInput', ...
              'Input "perf" must be a struct.');
    end
    if nargin < 2 || isempty(out_dir)
        out_dir = 'figures';
    end
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    [~, iT]  = min(perf.Tr);
    [~, iP]  = min(perf.Pr);
    [~, iLD] = max(perf.LD);

    navy  = [0.122, 0.306, 0.475];
    red   = [0.753, 0.224, 0.169];
    amber = [0.827, 0.329, 0.000];
    teal  = [0.067, 0.478, 0.396];

    specs = { ...
        '01_trim_alpha_vs_V.png',       perf.V,         perf.alpha_deg, 'Airspeed, V (m/s)',           'Trim Angle of Attack, \alpha (deg)',       'Trim Angle of Attack vs. Airspeed'; ...
        '02_trim_elevator_vs_alpha.png',perf.alpha_deg, perf.de_deg,    'Angle of Attack, \alpha (deg)','Trim Elevator Deflection, \delta_e (deg)', 'Trim Elevator Deflection vs. Angle of Attack'; ...
        '03_CL_vs_alpha.png',           perf.alpha_deg, perf.CL,        'Angle of Attack, \alpha (deg)','Lift Coefficient, C_L (-)',                'Trimmed Lift Coefficient vs. Angle of Attack'; ...
        '04_CD_vs_alpha.png',           perf.alpha_deg, perf.CD,        'Angle of Attack, \alpha (deg)','Drag Coefficient, C_D (-)',                'Drag Coefficient vs. Angle of Attack'; ...
        '05_thrust_required_vs_V.png',  perf.V,         perf.Tr,        'Airspeed, V (m/s)',           'Thrust Required, T_r (N)',                 'Thrust Required vs. Airspeed'; ...
        '06_power_required_vs_V.png',   perf.V,         perf.Pr,        'Airspeed, V (m/s)',           'Power Required, P_r (W)',                  'Power Required vs. Airspeed'; ...
        '07_CL_CD_vs_alpha.png',        perf.alpha_deg, perf.LD,        'Angle of Attack, \alpha (deg)','Lift-to-Drag Ratio, C_L / C_D (-)',        'Lift-to-Drag Ratio vs. Angle of Attack'; ...
        '08_CL32_CD_vs_alpha.png',      perf.alpha_deg, perf.CL32_CD,   'Angle of Attack, \alpha (deg)','Endurance Parameter, C_L^{3/2} / C_D (-)', 'Endurance Parameter vs. Angle of Attack' ...
    };

    for k = 1:size(specs, 1)
        fname  = fullfile(out_dir, specs{k, 1});
        x      = specs{k, 2};
        y      = specs{k, 3};
        xlab   = specs{k, 4};
        ylab   = specs{k, 5};
        tstr   = specs{k, 6};

        fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 900, 620]);
        plot(x, y, '-o', 'Color', navy, 'LineWidth', 1.8, ...
             'MarkerSize', 4.5, 'MarkerFaceColor', navy, 'MarkerEdgeColor', 'w');
        hold on;
        plot(x(iT),  y(iT),  'd', 'MarkerSize', 9, 'MarkerFaceColor', red,   'MarkerEdgeColor', 'w');
        plot(x(iLD), y(iLD), 'o', 'MarkerSize', 7, 'MarkerFaceColor', amber, 'MarkerEdgeColor', 'w');
        plot(x(iP),  y(iP),  's', 'MarkerSize', 8, 'MarkerFaceColor', teal,  'MarkerEdgeColor', 'w');

        grid on;
        xlabel(xlab, 'FontWeight', 'bold');
        ylabel(ylab, 'FontWeight', 'bold');
        title(tstr, 'FontWeight', 'bold');

        legend('Steady Trim Sweep', ...
               sprintf('Min Thrust (\\alpha=%.1f\\circ, V=%.2f m/s)', perf.alpha_deg(iT), perf.V(iT)), ...
               sprintf('Max L/D (%.3f at \\alpha=%.1f\\circ)', perf.LD(iLD), perf.alpha_deg(iLD)), ...
               sprintf('Min Power (%.2f W at \\alpha=%.1f\\circ)', perf.Pr(iP), perf.alpha_deg(iP)), ...
               'Location', 'best');

        text(x(iT), y(iT), sprintf('  Min T_r / Max L/D (\\alpha=%.1f\\circ)', perf.alpha_deg(iT)), ...
             'Color', red, 'FontSize', 8, 'VerticalAlignment', 'bottom');
        text(x(iP), y(iP), sprintf('  Min P_r (\\alpha=%.1f\\circ)', perf.alpha_deg(iP)), ...
             'Color', teal, 'FontSize', 8, 'VerticalAlignment', 'top');

        print(fig, fname, '-dpng', '-r300');
        close(fig);
    end
end
