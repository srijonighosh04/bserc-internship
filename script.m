%% =====================================================================
%  Trim, Thrust Required and Power Required Analysis
%  Cropped Delta Wing UAV (wing-alone), Steady Level Flight
%  =====================================================================
clear; clc; close all;

%% 1. INPUT DATA ---------------------------------------------------------
Cr = 0.9;  Ct = 0.15;  b = 1.5;  e_osw = 0.89;      % geometry [m]
CD0 = 0.03;  Cm0 = 0.01;  CL0 = 0;                   % CL0 assumed 0 (not given)
CLa = 2.92;  CLe = 0.265;                            % per rad
Cma = -0.292; Cme = -0.4;                            % per rad
m = 3.5;  rho = 1.225;  g = 10;                      % kg, kg/m^3, m/s^2
W = m*g;                                             % weight [N]

%% 2. PREPROCESSING ------------------------------------------------------
lambda = Ct/Cr;                    % taper ratio
S  = (b/2)*Cr*(1 + lambda);        % planform area [m^2]
AR = b^2/S;                        % aspect ratio
k  = 1/(pi*e_osw*AR);              % induced drag factor
fprintf('lambda = %.4f | S = %.4f m^2 | AR = %.4f | k = %.4f | W = %.1f N\n',...
        lambda, S, AR, k, W);

%% 3. ANGLE-OF-ATTACK SWEEP ---------------------------------------------
alpha_deg = 0:0.5:12;
alpha = deg2rad(alpha_deg);

[de, CL, V, CD, Tr, Pr, LD, CL32_CD] = trim_performance(alpha, ...
    CL0, CLa, CLe, Cm0, Cma, Cme, CD0, k, W, rho, S);

%% 4. RESULTS TABLE ------------------------------------------------------
T = table(alpha_deg', rad2deg(de)', CL', CD', V', Tr', Pr', LD', CL32_CD', ...
    'VariableNames', {'alpha_deg','delta_e_deg','CL','CD','V_mps', ...
                      'Tr_N','Pr_W','CL_CD','CL32_CD'});
disp(T);
writetable(T, 'uav_trim_results.csv');

[~,iT]  = min(Tr);  [~,iP] = min(Pr);
[~,iLD] = max(LD);  [~,iE] = max(CL32_CD);
fprintf('\nMin thrust required : %.3f N  at alpha = %.1f deg, V = %.2f m/s\n', Tr(iT), alpha_deg(iT), V(iT));
fprintf('Min power required  : %.3f W  at alpha = %.1f deg, V = %.2f m/s\n', Pr(iP), alpha_deg(iP), V(iP));
fprintf('Max CL/CD           : %.3f   at alpha = %.1f deg\n', LD(iLD), alpha_deg(iLD));
fprintf('Max CL^1.5/CD       : %.3f   at alpha = %.1f deg\n', CL32_CD(iE), alpha_deg(iE));

%% 5. PLOTS --------------------------------------------------------------
figure('Name','Trim & Performance','Position',[50 50 1200 800]);
subplot(3,3,1); plot(V, alpha_deg,'b-o','MarkerSize',3); grid on
xlabel('V (m/s)'); ylabel('Trim \alpha (deg)'); title('Trim angle of attack');
subplot(3,3,2); plot(alpha_deg, rad2deg(de),'r-o','MarkerSize',3); grid on
xlabel('\alpha (deg)'); ylabel('\delta_{e,trim} (deg)'); title('Trim elevator deflection');
subplot(3,3,3); plot(alpha_deg, CL,'k-o','MarkerSize',3); grid on
xlabel('\alpha (deg)'); ylabel('C_L (-)'); title('Lift coefficient');
subplot(3,3,4); plot(alpha_deg, CD,'m-o','MarkerSize',3); grid on
xlabel('\alpha (deg)'); ylabel('C_D (-)'); title('Drag coefficient');
subplot(3,3,5); plot(V, Tr,'b-o','MarkerSize',3); grid on
xlabel('V (m/s)'); ylabel('T_r (N)'); title('Thrust required');
subplot(3,3,6); plot(V, Pr,'r-o','MarkerSize',3); grid on
xlabel('V (m/s)'); ylabel('P_r (W)'); title('Power required');
subplot(3,3,7); plot(alpha_deg, LD,'g-o','MarkerSize',3); grid on
xlabel('\alpha (deg)'); ylabel('C_L/C_D (-)'); title('Lift-to-drag ratio');
subplot(3,3,8); plot(alpha_deg, CL32_CD,'c-o','MarkerSize',3); grid on
xlabel('\alpha (deg)'); ylabel('C_L^{3/2}/C_D (-)'); title('Endurance parameter');
subplot(3,3,9); plot(V, CL,'k-o','MarkerSize',3); grid on
xlabel('V (m/s)'); ylabel('C_L (-)'); title('C_L vs velocity');
saveas(gcf,'uav_trim_plots.png');

%% =====================================================================
%  LOCAL FUNCTION
%  =====================================================================
function [de, CL, V, CD, Tr, Pr, LD, CL32_CD] = trim_performance(alpha, ...
         CL0, CLa, CLe, Cm0, Cma, Cme, CD0, k, W, rho, S)
    de = -(Cm0 + Cma.*alpha)./Cme;              % Cm = 0  ->  delta_e,trim [rad]
    CL = CL0 + CLa.*alpha + CLe.*de;            % trimmed lift coefficient
    V  = sqrt(2*W./(rho*S.*CL));                % from L = W
    CD = CD0 + k.*CL.^2;                        % drag polar
    Tr = 0.5*rho.*V.^2*S.*CD;                   % thrust required (= D)
    Pr = Tr.*V;                                 % power required
    LD = CL./CD;                                % aerodynamic efficiency
    CL32_CD = CL.^1.5./CD;                      % endurance parameter
end