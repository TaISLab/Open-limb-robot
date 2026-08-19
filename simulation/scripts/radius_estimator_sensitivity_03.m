%% 03_radius_estimator_sensitivity.m
%
% On-Limb Orbiting Robot: Proprioceptive Diameter Estimation and
% Orthogonal Grip-Orbit Control
%
% Closed-form proprioceptive cylinder-radius and contact-geometry
% estimator (Section 4.3), with the sensitivity analysis of Section 8.2.
% Reproduces Figure 8: (i) required arm angle theta_a vs. cylinder
% diameter D, (ii) estimator sensitivities and combined sensor-limited
% error, (iii) lateral-contact angle gamma vs. D, (iv) radial-force
% lever d(rho)/d(gamma) vs. D.
%
% Equation numbers in comments refer to the manuscript.
% Requires the shared geometry functions in ../lib/ on the MATLAB/Octave path.
%
% Compatible with MATLAB and GNU Octave.

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'lib'));

%% ------------------------------------------------------------------
%  1. Design parameters (Table 1)
%  ------------------------------------------------------------------
rw = 19e-3;     % wheel radius [m]
l0 = 10e-3;     % lateral arm pivot x-offset [m]
l1 = 93e-3;     % lateral arm length (pivot to wheel centre) [m]
l3 = 30e-3;     % swing-link length [m]
l4 = 14e-3;     % swing-link pivot y-offset [m]

theta0_nom = [0 0];                       % unloaded swing links (nominal, Section 8.2)
a_nom = l4 + rw;                           % Eq. (1) with theta0 = 0 -> a = l4 + rw = 33 mm

%% ------------------------------------------------------------------
%  2. Physical limits of theta_a (Section 8.2)
%  ------------------------------------------------------------------
% Minimum diameter: the two lateral wheels collide, P1x = rw
theta_a_max = acos( (rw - l0) / l1 );      % = 84.4 deg, D_min = 45 mm

% Maximum diameter: lateral contact reaches the cylinder equator (gamma = 90 deg)
% Closed form, Eq. (47):
theta_a_min = acos( (rw - a_nom - l0) / (sqrt(2)*l1) ) - deg2rad(45);  % = 55.5 deg, D_max = 87 mm

fprintf('Valid arm-angle range: theta_a in [%.1f, %.1f] deg\n', ...
    rad2deg(theta_a_min), rad2deg(theta_a_max));

%% ------------------------------------------------------------------
%  3. Sweep theta_a over the valid range and compute D = 2*Rest
%  ------------------------------------------------------------------
N = 200;
theta_a_vec = linspace(theta_a_min, theta_a_max, N);

Rest_vec  = zeros(1, N);
gamma_vec = zeros(1, N);
for k = 1:N
    Rest_vec(k)  = radiusEstimator(theta_a_vec(k), theta0_nom, l0, l1, l3, l4, rw);
    gamma_vec(k) = lateralContactAngle(theta_a_vec(k), theta0_nom, l0, l1, l3, l4, rw);
end
D_vec = 2*Rest_vec;

% Sort by increasing diameter for plotting (theta_a decreases as D increases)
[D_vec, idx] = sort(D_vec);
theta_a_vec  = theta_a_vec(idx);
gamma_vec    = gamma_vec(idx);

fprintf('Resulting diameter range: D in [%.1f, %.1f] mm\n\n', D_vec(1)*1e3, D_vec(end)*1e3);

%% ------------------------------------------------------------------
%  4. Sensitivity analysis (Section 8.2, Eqs. 48-49)
%  ------------------------------------------------------------------
dtheta_a  = deg2rad(0.1);   % encoder resolution [rad], delta_theta_a
dtheta_0  = deg2rad(1.0);   % swing-link sensor resolution [rad], delta_theta_0,i (per sensor)

dR_dthetaa   = zeros(1, N);
dR_dtheta0   = zeros(1, N);   % combined |dR/dtheta0f| + |dR/dtheta0b|
dR_total     = zeros(1, N);

h = 1e-6;  % finite-difference step [rad], for numerical partials (cross-check of Eq. 48)

for k = 1:N
    th_a = theta_a_vec(k);

    % --- partial wrt theta_a (central finite difference) ---
    Rp = radiusEstimator(th_a + h, theta0_nom, l0, l1, l3, l4, rw);
    Rm = radiusEstimator(th_a - h, theta0_nom, l0, l1, l3, l4, rw);
    dR_dthetaa(k) = abs( (Rp - Rm) / (2*h) );

    % --- partial wrt each swing-link angle, Eq. (48) closed form ---
    P1x = l0 + l1*cos(th_a);
    a0  = contactDepth(theta0_nom, l3, l4, rw);
    u   = -l1*sin(th_a) + a0;
    dR_dtheta0_f = (P1x^2 - (u - rw)^2) / (2*(u - rw)^2) * (l3/2) * cos(theta0_nom(1));
    dR_dtheta0_b = (P1x^2 - (u - rw)^2) / (2*(u - rw)^2) * (l3/2) * cos(theta0_nom(2));
    dR_dtheta0(k) = abs(dR_dtheta0_f) + abs(dR_dtheta0_b);

    % --- combined first-order error, Eq. (49) ---
    dR_total(k) = dR_dthetaa(k)*dtheta_a + dR_dtheta0(k)*dtheta_0;
end

% convert sensitivities to mm/deg for plotting, matching Figure 8
dR_dthetaa_mmdeg = dR_dthetaa * (pi/180) * 1e3;
dR_dtheta0_mmdeg = dR_dtheta0 * (pi/180) * 1e3;
dR_total_mm      = dR_total * 1e3;

fprintf('Peak total radius error: %.2f mm (tolerance: 1 mm)\n\n', max(dR_total_mm));

%% ------------------------------------------------------------------
%  5. Radial-force lever d(rho)/d(gamma), Eq. (10)
%  ------------------------------------------------------------------
drho_dgamma_mmdeg = zeros(1, N);
for k = 1:N
    th_a = theta_a_vec(k);
    R = radiusEstimator(th_a, theta0_nom, l0, l1, l3, l4, rw);
    g  = gamma_vec(k);
    % Eq. (10): drho/dgamma = (R+rw) * (sin(gamma) - cos(gamma)*cot(theta_a))
    val = (R + rw) * ( sin(g) - cos(g)/tan(th_a) );
    drho_dgamma_mmdeg(k) = val * (pi/180) * 1e3;   % convert per-rad -> per-deg, m -> mm
end

%% ------------------------------------------------------------------
%  6. Reproduce Figure 8 (four stacked subplots vs. cylinder diameter)
%  ------------------------------------------------------------------
D_mm = D_vec * 1e3;

figure('Name','Radius estimator sensitivity analysis (Figure 8)');

subplot(4,1,1);
plot(D_mm, rad2deg(theta_a_vec), 'LineWidth', 1.5); grid on;
ylabel('\theta_a [deg]');
title('Sensitivity analysis of the online radius estimator (Eq. 3)');

subplot(4,1,2);
plot(D_mm, dR_dthetaa_mmdeg, 'b-', 'LineWidth', 1.5); hold on;
plot(D_mm, dR_dtheta0_mmdeg, 'r--', 'LineWidth', 1.5);
plot(D_mm, dR_total_mm, 'Color', [0.9 0.6 0], 'LineWidth', 1.5);
plot(D_mm, ones(size(D_mm)), 'k--', 'LineWidth', 1.2);  % tolerance line (yline avoided for Octave compatibility)
grid on; legend('|\partial R/\partial\theta_a|', '\Sigma_i|\partial R/\partial\theta_{0,i}|', ...
    '\delta R total [mm]', 'tolerance', 'Location', 'best');
ylabel('[mm/deg] / [mm]');

subplot(4,1,3);
plot(D_mm, rad2deg(gamma_vec), 'LineWidth', 1.5); grid on;
plot(D_mm, 90*ones(size(D_mm)), 'k:');  % gamma=90 deg limit (yline avoided for Octave compatibility)
ylabel('\gamma [deg]');

subplot(4,1,4);
plot(D_mm, drho_dgamma_mmdeg, 'LineWidth', 1.5); grid on;
xlabel('Cylinder diameter D [mm]');
ylabel('d\rho/d\gamma [mm/deg]');
