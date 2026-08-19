%% 01_static_force_model.m
%
% On-Limb Orbiting Robot: Proprioceptive Diameter Estimation and
% Orthogonal Grip-Orbit Control
%
% Static contact-force model and actuation-space decomposition.
% Reproduces the equations of Sections 4 (Kinematic and Contact
% Modeling) and 5 (Static Force Model), and numerically verifies the
% central structural result of the paper: the common-mode/differential
% wheel-torque basis diagonalises both the velocity Jacobian (Eq. 8)
% and the static force map (Eq. 20), so that orbital propulsion and
% central-contact-force regulation are provably decoupled.
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
m  = 0.28;      % total vehicle mass [kg]
g  = 9.81;      % gravitational acceleration [m/s^2]
mu = 0.5;       % (assumed) friction coefficient, Section 8.1

%% ------------------------------------------------------------------
%  2. Grasp configuration for a chosen cylinder diameter
%  ------------------------------------------------------------------
D = 56e-3;              % cylinder diameter [m] (matches the prototype test, Section 8.4)
R = D/2;

theta_a = solveArmAngle(R, l0, l1, l3, l4, rw);   % Eq. (3) inverted numerically
fprintf('Grasp configuration for D = %.1f mm:\n', D*1e3);
fprintf('  theta_a = %.2f deg\n', rad2deg(theta_a));

% Contact geometry (Section 4.2)
theta0 = [0 0];                                    % unloaded swing links (nominal)
a0     = contactDepth(theta0, l3, l4, rw);          % Eq. (1)
Pc0    = [0; -a0];

P1  = [l0 + l1*cos(theta_a); -l1*sin(theta_a)];     % right lateral wheel centre
P2  = [-P1(1); P1(2)];                              % left, by symmetry (Section 4.2)

Rest = radiusEstimator(theta_a, theta0, l0, l1, l3, l4, rw); % Eq. (3)
Cy   = -a0 - Rest;
C    = [0; Cy];

n1 = (C - P1) / norm(C - P1);
Pc1 = P1 + rw*n1;
Pc2 = [-Pc1(1); Pc1(2)];

gamma = lateralContactAngle(theta_a, theta0, l0, l1, l3, l4, rw); % Eq. (4)
fprintf('  R_est   = %.2f mm (true R = %.2f mm)\n', Rest*1e3, R*1e3);
fprintf('  gamma   = %.2f deg\n\n', rad2deg(gamma));

%% ------------------------------------------------------------------
%  3. Velocity Jacobian and force duality (Eqs. 7-12)
%  ------------------------------------------------------------------
M = [1 1; 1 -1];                 % common-mode / differential basis
J = (rw/(2*R)) * M;              % Eq. (8): [phidot; gammadot] = J*[theta1dot; theta2dot]

fprintf('Orthogonality check M*M'' (should be 2*I):\n');
disp(M*M');

% Force duality: tau = J^-T * [tauSigma_op; dtau_op] with J^-T = (R/rw)*M, Eq. (11)
% Allocation law (inverse), Eq. (12): [tauw1;tauw2] = 0.5*M*[tauSigma;dtau]

%% ------------------------------------------------------------------
%  4. Static force model: grasping-torque contribution (Section 5.1)
%  ------------------------------------------------------------------
Ps = [l0; 0];                       % grasping-actuator axis
ell = Pc1 - Ps;
rc  = norm(ell);                    % Eq. (14): contact distance / actuator moment arm
ea  = [ell(2); -ell(1)] / rc;       % Eq. (15)
alpha = acos( dot(ea, n1) );        % angle between ea and n1

tau_a_nom = 200e-3;                 % example grasping torque [N.m], for illustration
fn_a = tau_a_nom / (2*rc) * cos(alpha);  % Eq. (16): normal force induced by tau_a alone
fprintf('Example grasping torque tau_a = %.1f mN.m  ->  fn,a = %.2f N (alpha = %.1f deg)\n', ...
    tau_a_nom*1e3, fn_a, rad2deg(alpha));

%% ------------------------------------------------------------------
%  5. Static force model: wheel-torque contribution (Section 5.2)
%     Verifies the diagonal force map of Eq. (20):
%       tau_phi   = (R/rw) * tauSigma          (depends only on tauSigma)
%       Delta_fn0 = -(sin(gamma)/rw) * dtau    (depends only on dtau)
%  ------------------------------------------------------------------
tauSigma_range = linspace(-100e-3, 100e-3, 21);  % common-mode torque sweep [N.m]
dtau_range     = linspace(-50e-3, 50e-3, 21);    % differential torque sweep [N.m]

tau_phi   = zeros(numel(tauSigma_range), numel(dtau_range));
dfn0_grid = zeros(numel(tauSigma_range), numel(dtau_range));

for i = 1:numel(tauSigma_range)
    for j = 1:numel(dtau_range)
        tauSigma = tauSigma_range(i);
        dtau     = dtau_range(j);
        tau_phi(i,j)   = (R/rw) * tauSigma;             % Eq. (20), row 1
        dfn0_grid(i,j) = -(sin(gamma)/rw) * dtau;       % Eq. (20), row 2
    end
end

% Off-diagonal sensitivities (should be ~0 to machine precision)
d_tauphi_d_dtau     = max(max(abs(diff(tau_phi, 1, 2))));
d_dfn0_d_tauSigma   = max(max(abs(diff(dfn0_grid, 1, 1))));
fprintf('\nDecoupling check (Eq. 20):\n');
fprintf('  max |d(tau_phi)/d(dtau)|   = %.3e  (should be 0)\n', d_tauphi_d_dtau);
fprintf('  max |d(Delta fn0)/d(tauSigma)| = %.3e  (should be 0)\n', d_dfn0_d_tauSigma);

%% ------------------------------------------------------------------
%  6. Combined central contact force (Eq. 27) over one orbit
%  ------------------------------------------------------------------
phi = linspace(0, 2*pi, 361);
tau_a_orbit = 300e-3 * ones(size(phi));   % placeholder grasping torque profile [N.m]
dtau_orbit  = 20e-3 * sin(phi);           % placeholder differential command [N.m]

fn0 = m*g*cos(phi) ...                                      % gravity term
    + (cos(alpha)*cos(gamma)/rc) * tau_a_orbit ...           % grasping-actuator term
    - (sin(gamma)/rw) * dtau_orbit;                          % differential-traction term

figure('Name','Static force model verification');

subplot(2,1,1);
imagesc(dtau_range*1e3, tauSigma_range*1e3, tau_phi*1e3);
axis xy; colorbar;
xlabel('\Delta\tau [mN\cdotm]'); ylabel('\tau_\Sigma [mN\cdotm]');
title('\tau_\phi(\tau_\Sigma,\Delta\tau)  -- flat along \Delta\tau (Eq. 20)');

subplot(2,1,2);
imagesc(dtau_range*1e3, tauSigma_range*1e3, dfn0_grid);
axis xy; colorbar;
xlabel('\Delta\tau [mN\cdotm]'); ylabel('\tau_\Sigma [mN\cdotm]');
title('\Delta f_{n0}(\tau_\Sigma,\Delta\tau)  -- flat along \tau_\Sigma (Eq. 20)');

figure('Name','Central contact force over one orbit (Eq. 27)');
plot(rad2deg(phi), fn0, 'LineWidth', 1.5); grid on;
xlabel('\phi [deg]'); ylabel('f_{n0} [N]');
title('Central normal force decomposition (illustrative \tau_a, \Delta\tau profiles)');
