function fig = plot_results(perf, output_file)
%PLOT_RESULTS Generate 3x3 trim and performance figure and optionally save PNG.
%
%   fig = plot_results(perf) creates a 3x3 subplot figure summarizing trim
%   and steady level-flight performance metrics.
%
%   fig = plot_results(perf, output_file) also saves the figure as a PNG
%   image to the specified path using print('-dpng').
%
%   Compatible with MATLAB and GNU Octave.

    if nargin < 1 || ~isstruct(perf)
        error('plot_results:InvalidInput', 'Input "perf" must be a struct.');
    end

    fig = figure('Name', 'Trim & Performance', 'Position', [50, 50, 1200, 800]);

    subplot(3, 3, 1);
    plot(perf.V, perf.alpha_deg, 'b-o', 'MarkerSize', 3);
    grid on;
    xlabel('V (m/s)');
    ylabel('Trim \alpha (deg)');
    title('Trim angle of attack');

    subplot(3, 3, 2);
    plot(perf.alpha_deg, perf.de_deg, 'r-o', 'MarkerSize', 3);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('\delta_{e,trim} (deg)');
    title('Trim elevator deflection');

    subplot(3, 3, 3);
    plot(perf.alpha_deg, perf.CL, 'k-o', 'MarkerSize', 3);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L (-)');
    title('Lift coefficient');

    subplot(3, 3, 4);
    plot(perf.alpha_deg, perf.CD, 'm-o', 'MarkerSize', 3);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_D (-)');
    title('Drag coefficient');

    subplot(3, 3, 5);
    plot(perf.V, perf.Tr, 'b-o', 'MarkerSize', 3);
    grid on;
    xlabel('V (m/s)');
    ylabel('T_r (N)');
    title('Thrust required');

    subplot(3, 3, 6);
    plot(perf.V, perf.Pr, 'r-o', 'MarkerSize', 3);
    grid on;
    xlabel('V (m/s)');
    ylabel('P_r (W)');
    title('Power required');

    subplot(3, 3, 7);
    plot(perf.alpha_deg, perf.LD, 'g-o', 'MarkerSize', 3);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L/C_D (-)');
    title('Lift-to-drag ratio');

    subplot(3, 3, 8);
    plot(perf.alpha_deg, perf.CL32_CD, 'c-o', 'MarkerSize', 3);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L^{3/2}/C_D (-)');
    title('Endurance parameter');

    subplot(3, 3, 9);
    plot(perf.V, perf.CL, 'k-o', 'MarkerSize', 3);
    grid on;
    xlabel('V (m/s)');
    ylabel('C_L (-)');
    title('C_L vs velocity');

    if nargin >= 2 && ~isempty(output_file)
        print(fig, output_file, '-dpng', '-r150');
    end
end
