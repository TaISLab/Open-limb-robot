% =========================================================================
%  orbital_simulation.m  —  v7 (feedforward + optional PI compliance)
%  Closed-loop orbital simulation of the on-limb robot
%
%  Central radial-force architecture:
%    Delta_tau_d = Delta_tau_ff(phi) + [has_force_sensor]*Delta_tau_PI(t)
%  The command is the DIFFERENTIAL wheel torque directly [N.m]; there is no
%  intermediate dimensionless u0 nor a tau_c unit. Delta_tau_ff is a purely
%  model-based feedforward (always active; needs only phi, fn1 and gamma,
%  already available), and Delta_tau_PI is an additive PI correction on the
%  fn0 error, active ONLY when the compliant central contact is measured.
%  Geometric gain (eq. N0_affine):  d fn0 / d Delta_tau = -sin(gamma)/r_w.
%  Grasping normal force (eq. normal_from_actuator): fn1 = tau_a cos(alpha)/(2 r_c).
%  Channels are exactly decoupled: sum = orbital (tau_Sigma), diff = radial.
%
%  Estado: x = [phi; phi_dot; ei_phi; ei_fn0]  (4 estados)
%    ei_fn0 integrates the fn0 tracking error for the optional PI; it is
%    harmless (unused) when has_force_sensor = false.
% =========================================================================
clear; clc; close all;

%% ── PARAMETROS DEL ROBOT ────────────────────────────────────────────────
r_w  = 19e-3;
l_0  = 10e-3;
l_1  = 93e-3;
l_3  = 30e-3;
l_4  = 14e-3;
l_0_central = 0;     % central wheel on y-axis
a    = l_4 + r_w;        % central contact depth P_c0=(0,-a): l4+rw [m]
m    = 0.28;   % measured prototype mass [kg]
g    = 9.81;
mu   = 0.5;
b_v  = 2e-4;
theta_a = 65.5*pi/180;   % arm angle [rad], positive downward; D≈66mm cylinder

% Cylinder geometry from estimator (eq. radius_est):
P1x = l_0 + l_1*cos(theta_a);
P1y = -l_1*sin(theta_a);
R_c = (r_w^2 - P1x^2 - (P1y+a)^2) / (2*(P1y+a-r_w));  % estimated radius [m]
C_y = -a - R_c;                                           % cylinder axis y
% Contact frame at the right lateral wheel:
n1_vec = ([0; C_y] - [P1x; P1y]) / norm([0; C_y] - [P1x; P1y]);
Pc1    = [P1x; P1y] + r_w*n1_vec;
Ps     = [l_0; 0];
% r_c = contact distance = moment arm of tau_a about the actuator axis Ps
r_c    = norm(Pc1 - Ps);
% gamma = lateral-contact angle (eq. gamma_angle)
gamma  = asin(P1x/(R_c + r_w));
% cos(alpha) = projection of the actuator force onto the contact normal
%             (eq. normal_from_actuator): e_a = unit vector perp. to lever arm
ell    = Pc1 - Ps;
e_a    = [ell(2); -ell(1)]/r_c;
if dot(e_a, n1_vec) < 0, e_a = -e_a; end
cos_alpha = dot(e_a, n1_vec);
d_G    = abs(C_y);
I_zz   = m * d_G^2;
fprintf('theta_a=%.1fdeg  R_c=%.1fmm  D=%.1fmm  r_c=%.1fmm  gamma=%.1fdeg  cos_a=%.3f  d_G=%.1fmm\n',...
        theta_a*180/pi, R_c*1e3, 2*R_c*1e3, r_c*1e3, gamma*180/pi, cos_alpha, d_G*1e3);

%% ── TRAYECTORIA ─────────────────────────────────────────────────────────
t_end       = 20;
phi_target  = 2*pi;
phi_dot_max = phi_target / (0.9*t_end);

%% ── PARAMETROS DE CONTROL ───────────────────────────────────────────────
Kp_phi    = 8.0;
Ki_phi    = 0.5;
tau_a_min = 100e-3;
tau_a_max = 1500e-3;
% ── ARQUITECTURA DE FUERZA RADIAL: FEEDFORWARD + PI OPCIONAL ─────────────
% El comando es directamente el par diferencial Delta_tau_d [N.m] (canal
% diferencial). No hay variable u0 intermedia ni unidad tau_c: la ganancia
% geometrica es sin(gamma)/r_w (eq. N0_affine), constante para un agarre dado.
has_force_sensor = true;      % true: PI activo (requiere sensor dedicado)
                              % false: solo feedforward, sin correccion
Delta_tau_max = 50e-3;        % [N.m] saturacion del par diferencial
Kp_fn0  = 20e-3;  % [N.m/N]    ganancia P del PI de fuerza
Ki_fn0  = 8e-3;   % [N.m/(N.s)] ganancia I del PI de fuerza

% Incertidumbre que justifica el sensor de fuerza (afecta solo al termino FF
% y a la medida usada por el PI; el resultado fisico usa siempre la ganancia real):
meas_noise_std = 0.15;   % [N] ruido de medida en fn0 (solo si has_force_sensor)
k_comp_error   = 0.30;   % error relativo de la ganancia radial en el modelo FF

% Umbrales / objetivo de fn0 en Newtons (fuerza central AGRUPADA = 2 muelles).
% El banco calibro un solo muelle; los limites del contacto central son ~2x.
% El limite inferior es un piso de medida (precarga de los muelles), no un
% umbral de deslizamiento. No hay 'yield': el maximo es el rango medible.
fn0_min = 1.0;    % [N] fuerza minima de asentamiento (piso de medida)
fn0_max = 6.0;    % [N] fuerza maxima medible
% Objetivo NO en el centro geometrico: se elige dentro del alcance del canal
% diferencial (+/- sin(gamma)/r_w * Delta_tau_max ~ +/-2.5 N) alrededor de la
% linea base natural, de modo que Delta_tau CAMBIE DE SIGNO a lo largo de la
% vuelta (self-closing / self-opening) y tau_a pueda bajar a su minimo.
fn0_d   = 3.0;

%% ── PACK ────────────────────────────────────────────────────────────────
P.r_w=r_w; P.R_c=R_c; P.r_c=r_c; P.gamma=gamma; P.cos_alpha=cos_alpha;
P.m=m; P.g=g; P.d_G=d_G; P.I_zz=I_zz; P.b_v=b_v; P.mu=mu;
P.Kp=Kp_phi; P.Ki=Ki_phi;
P.tau_a_min=tau_a_min; P.tau_a_max=tau_a_max;
P.theta_a=theta_a; P.a=a;
P.fn0_min=fn0_min; P.fn0_max=fn0_max; P.fn0_d=fn0_d;
P.has_force_sensor=has_force_sensor; P.Delta_tau_max=Delta_tau_max;
P.Kp_fn0=Kp_fn0; P.Ki_fn0=Ki_fn0;
P.meas_noise_std=meas_noise_std; P.k_comp_error=k_comp_error;
P.t_end=t_end; P.phi_dot_max=phi_dot_max;

%% ── INTEGRACION ─────────────────────────────────────────────────────────
% Estado: x = [phi; phi_dot; ei_phi; ei_fn0]
x0   = [0; 0; 0; 0];
opts = odeset('MaxStep',0.02,'RelTol',1e-5,'AbsTol',1e-8,...
              'Events',@(t,x) evStop(t,x,phi_target));
[t, X] = ode45(@(t,x) odefun(t,x,P), [0 t_end], x0, opts);

phi  = X(:,1);
dphi = X(:,2);
fprintf('Fin: t=%.2fs  phi=%.1f deg\n', t(end), phi(end)*180/pi);
if has_force_sensor
    fprintf('Compliance: FEEDFORWARD + PI (dedicated force sensor)\n');
else
    fprintf('Compliance: FEEDFORWARD ONLY (no dedicated force sensor)\n');
end

%% ── RECONSTRUCCION ───────────────────────────────────────────────────────
N = length(t);
tw1=zeros(N,1); tw2=zeros(N,1); ta=zeros(N,1);
fn1=zeros(N,1); fn0=zeros(N,1); dtauv=zeros(N,1); tgv=zeros(N,1);
fn0_base=zeros(N,1); dtauff_v=zeros(N,1); dtaupi_v=zeros(N,1);
ta_slip_v=zeros(N,1); ta_stab_v=zeros(N,1);
for k=1:N
    [tw1(k),tw2(k),ta(k),fn1(k),fn0(k),dtauv(k),tgv(k),fn0_base(k),dtauff_v(k),dtaupi_v(k),ta_slip_v(k),ta_stab_v(k)] = ...
        ctrl(t(k), phi(k), dphi(k), X(k,3), X(k,4), P);
end

err = fn0 - fn0_d;
fprintf('fn0 range: %.2f to %.2f N  (target=%.2fN, band=[%.2f,%.2f]N)\n', ...
    min(fn0), max(fn0), fn0_d, fn0_min, fn0_max);
fprintf('fn0 tracking error: mean=%.3fN std=%.3fN max|e|=%.3fN\n', ...
    mean(err), std(err), max(abs(err)));
fprintf('Samples within admissible band: %.1f%%\n', ...
    100*mean(fn0>=fn0_min & fn0<=fn0_max));

%% ── FIGURAS ─────────────────────────────────────────────────────────────
c1=[0.18 0.45 0.70]; c2=[0.85 0.33 0.10];
c3=[0.47 0.67 0.19]; c4=[0.63 0.08 0.16];

figure('Name','Orbital simulation','Units','centimeters','Position',[2 2 18 24]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile; hold on;
plot(t, phi*180/pi,'Color',c1,'LineWidth',1.3);
yline(360,'--k','LineWidth',0.8,'Label','360°');
ylabel('\phi (°)'); title('Orbital angle');
xlim([0 t(end)]); ylim([0 400]); grid on;

% Orbital-only torque = total MINUS the differential compliance offset
% (+/- 0.5*u0*tau_c, OPPOSITE sign on each wheel per the Jacobian split):
% recovers the common-mode gravity feedforward tw_ff on each wheel. The
% compliance offset cancels in the sum tw1+tw2 = tau_Sigma (orbital channel)
% and appears only in the difference tw1-tw2 (fn0 channel).
tw1_orbital_only = tw1 - 0.5*dtauv;
tw2_orbital_only = tw2 + 0.5*dtauv;

nexttile; hold on;
% TOTAL commanded torques (solid, thick): orbital feedforward + compliance
h_tw1 = plot(t, tw1*1e3, 'LineStyle','-', 'Color',c2, 'LineWidth',1.6, ...
             'DisplayName','\tau_{w1} (total: orbital + radial force)');
h_tw2 = plot(t, tw2*1e3, 'LineStyle','-', 'Color',c1, 'LineWidth',1.6, ...
             'DisplayName','\tau_{w2} (total: orbital + radial force)');
% ORBITAL-ONLY (dashed, thin): total minus compliance, exactly
h_tw1g = plot(t, tw1_orbital_only*1e3, 'LineStyle','--', 'Color',c2, 'LineWidth',1.0, ...
              'DisplayName','\tau_{w1}^{orbital} (no radial-force)');
h_tw2g = plot(t, tw2_orbital_only*1e3, 'LineStyle','--', 'Color',c1, 'LineWidth',1.0, ...
              'DisplayName','\tau_{w2}^{orbital} (no radial-force)');
yline(0,':k','LineWidth',0.5,'HandleVisibility','off');
ylabel('\tau_w (N·mm)'); title('Lateral wheel torques');
legend([h_tw1,h_tw2,h_tw1g,h_tw2g],'Location','northeast','FontSize',8,'NumColumns',2);
xlim([0 t(end)]); grid on;

nexttile; hold on;
% Candidate lower bounds whose upper envelope IS tau_a* (eq. tau_ff):
%   ta_slip    -> slip prevention (Coulomb), dominated by the orbital torque
%   ta_central -> central non-detachment, dominated by gravity at phi=180 deg
h_slip = plot(t, ta_slip_v*1e3,'Color',[0.60 0.60 0.60],'LineWidth',0.8,'LineStyle','-');
h_stab = plot(t, ta_stab_v*1e3,'Color',[0.30 0.55 0.75],'LineWidth',0.8,'LineStyle',':');
h_taus = plot(t, ta*1e3,'Color',c4,'LineWidth',1.5);   % final command (envelope)
yline(tau_a_min*1e3,'--k','LineWidth',0.8,'Label','\tau_{a,min}');
ylabel('\tau_a (N·mm)'); title('Grasping torque \tau_a^* = max(\tau_a^{slip}, \tau_a^{stab}, \tau_{a,min})');
legend([h_taus,h_slip,h_stab], ...
       {'\tau_a^* (command)','\tau_a^{slip} (Coulomb / orbital)','\tau_a^{stab} (non-detachment / gravity)'}, ...
       'Location','north','FontSize',8,'NumColumns',3);
ylim([0 1350]);   % show the full peak (~1.3 N.m)
xlim([0 t(end)]); grid on;

nexttile; hold on;
yyaxis left;
h_fn1 = plot(t, fn1,'Color',c2,'LineWidth',1.2);
h_fn0 = plot(t, fn0,'-','Color',c1,'LineWidth',1.4);
h_fn0b= plot(t, fn0_base,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);
yline(fn0_min, '--','Color',c2,'LineWidth',0.9,'HandleVisibility','off');
yline(fn0_max,'--','Color',c1,'LineWidth',0.9,'HandleVisibility','off');
yline(fn0_d,     ':','Color',[0.3 0.3 0.3],'LineWidth',0.9,'HandleVisibility','off');
y_max = max([max(fn1), max(fn0), fn0_max]) * 1.20;
y_min = min(0, min(fn0_base)) * 1.15;
ylim([y_min y_max]);
text(t(end)*0.50, fn0_min + (y_max-y_min)*0.03, 'f_{n0,min}', 'FontSize',8,'Color',c2);
text(t(end)*0.50, fn0_max + (y_max-y_min)*0.03, 'f_{n0,max}','FontSize',8,'Color',c1);
yline(0,':k','LineWidth',0.5,'HandleVisibility','off');
ylabel('f_n (N)');
yyaxis right;
h_u0 = plot(t, dtauv*1e3,'Color',[0.4 0.4 0.4],'LineWidth',1.2);
ylabel('\Delta\tau_d (N\cdotmm)'); ylim([-Delta_tau_max*1e3*1.1 Delta_tau_max*1e3*1.1]);
legend([h_fn1, h_fn0, h_fn0b, h_u0], ...
       {'f_{n1} lateral','f_{n0} (FF+PI)','f_{n0} baseline (no radial-force)','\Delta\tau_d differential'}, ...
       'Location','northeast','FontSize',8);
title('Contact forces and radial-force command');
xlabel('Time (s)'); xlim([0 t(end)]); grid on;

set(findall(gcf,'-property','FontSize'),'FontSize',9);
exportgraphics(gcf,'orbital_simulation.pdf','ContentType','vector');
fprintf('Guardado orbital_simulation.pdf\n');

%% =========================================================================
%  FUNCIONES
%% =========================================================================

function dx = odefun(t, x, P)
    phi=x(1); dphi=x(2); ei=x(3); ei_fn0=x(4);
    [tw1,tw2,ta,~,fn0_now,~,tg,~,~,~] = ctrl(t,phi,dphi,ei,ei_fn0,P);
    fn1_est   = ta*P.cos_alpha/(2*P.r_c);   % eq. normal_from_actuator
    tf        = P.b_v*dphi + 2*fn1_est*0.01*P.R_c*sign(dphi+1e-9);
    tau_sigma = tw1 + tw2;   % common-mode is the orbital driving channel (eq. wheel_torque_effects)
    phi_ddot  = ((P.R_c/P.r_w)*tau_sigma - tg - tf) / P.I_zz;
    dphi_d    = trapz_vel(t, P.t_end, P.phi_dot_max);
    e_dphi    = dphi_d - dphi;

    % Error de fn0 para el integrador del PI opcional (siempre se integra;
    % solo se USA si has_force_sensor=true, por lo que es inocuo en caso contrario)
    e_fn0 = P.fn0_d - fn0_now;

    dx = [dphi; phi_ddot; e_dphi; e_fn0];
end

function [tw1,tw2,ta,fn1,fn0,dtau_d,tg,fn0_base,dtau_ff,dtau_pi,ta_slip,ta_stab] = ctrl(t,phi,dphi,ei,ei_fn0,P)
    % Gravitational bias
    tg = -P.m * P.g * P.d_G * sin(phi);

    % Orbital PI + gravity feedforward -> COMMON-MODE torque (drives the orbit)
    dphi_d    = trapz_vel(t, P.t_end, P.phi_dot_max);
    e_dphi    = dphi_d - dphi;
    tau_phi_d = tg + P.Kp*e_dphi + P.Ki*ei;
    tau_sigma_ff = (P.r_w/P.R_c) * tau_phi_d;   % eq. delta_tau_ctrl
    tw1_ff    =  tau_sigma_ff/2;
    tw2_ff    =  tau_sigma_ff/2;                 % SAME sign: gravity FF is common-mode

    % ── GRASPING TORQUE FEEDFORWARD (two cascaded stages) ────────────────
    % Note: fn1 = tau_a*cos_alpha/(2 r_c)  (eq. normal_from_actuator), so
    %       tau_a = 2 r_c fn1 / cos_alpha.  The actuator gain per unit tau_a is
    ka = P.cos_alpha/(2*P.r_c);                  % fn1 per unit tau_a
    % geometric coupling of the differential channel onto fn0 (eq. N0_affine):
    k_comp_true = sin(P.gamma)/P.r_w;            % |d fn0 / d Delta_tau|  [N per N.m]

    %  (i) Stage 1 — contact stability (no detachment), eq. tau_stab:
    ta_lateral = (2*P.r_c/P.cos_alpha)*(abs(tg)/(2*P.mu*P.R_c) - cos(phi)*P.m*P.g*0.5);
    %  central floor: raise tau_a ONLY when, even at tau_a_min, the natural
    %  baseline would drop below the differential's reach of the target,
    %  i.e. below fn0_d - Delta_tau_max*(sin gamma/r_w). Above that, tau_a is
    %  left free (the differential alone regulates fn0), so tau_a can fall to
    %  its minimum and Delta_tau is free to change sign.
    fn1_min      = P.tau_a_min*ka;
    base_at_min  = P.m*P.g*cos(phi) + 2*fn1_min*cos(P.gamma);
    base_needed  = P.fn0_d - P.Delta_tau_max*k_comp_true;
    if base_at_min < base_needed
        ta_central = (base_needed - P.m*P.g*cos(phi))/(2*cos(P.gamma)*ka);
    else
        ta_central = 0;
    end
    ta_stab = max([0, ta_lateral, ta_central]);

    ta_s1  = clamp(ta_stab, P.tau_a_min, P.tau_a_max);
    fn1_s1 = ta_s1 * ka;

    % differential command estimate at Stage-1 preload (needed for Stage 2)
    fn0_base_s1 = P.m*P.g*cos(phi) + 2*fn1_s1*cos(P.gamma);
    k_model     = k_comp_true * (1 + P.k_comp_error);
    dtau_ff_s1  = clamp((P.fn0_d - fn0_base_s1)/(-k_model), -P.Delta_tau_max, P.Delta_tau_max);

    %  (iii) Stage 2 — slip prevention (Coulomb), eq. tau_slip:
    %   tw1 = tau_sigma/2 + Delta_tau/2, tw2 = tau_sigma/2 - Delta_tau/2
    %   ta_slip = (2 r_c / cos_alpha) * max(|tw1|,|tw2|)/(mu r_w)
    %           = (r_c / cos_alpha) * (|tau_sigma| + |Delta_tau|)/(mu r_w)
    ta_slip = (P.r_c/P.cos_alpha)*(abs(tau_sigma_ff) + abs(dtau_ff_s1))/(P.mu*P.r_w);

    ta_ff = max(ta_stab, ta_slip);
    ta    = clamp(ta_ff, P.tau_a_min, P.tau_a_max);
    fn1   = ta * ka;                             % eq. normal_from_actuator

    % Clamp the two candidates for plotting so that
    % max(ta_slip, ta_stab, tau_a_min) reproduces the command ta exactly.
    % ta_stab already bundles lateral non-lift-off and central non-detachment.
    ta_slip = clamp(ta_slip, 0, P.tau_a_max);
    ta_stab = clamp(ta_stab, 0, P.tau_a_max);

    % Baseline fn0 (no radial-force command): eq. fn0_baseline
    fn0_base = P.m*P.g*cos(phi) + 2*fn1*cos(P.gamma);

    % ── FEEDFORWARD RADIAL-FORCE COMMAND (eq. u0_ff) — always active ──────
    % Delta_tau_ff = -(r_w/sin gamma) (fn0_d - fn0_base); model gain imperfect
    dtau_ff = (P.fn0_d - fn0_base)/(-k_model);
    dtau_ff = clamp(dtau_ff, -P.Delta_tau_max, P.Delta_tau_max);

    % ── OPTIONAL PI CORRECTION (eq. u0_PI) — only if force sensor present ─
    if P.has_force_sensor
        pseudo_noise = P.meas_noise_std * ( ...
                        0.6*sin(9*t) + 0.3*sin(23*t+0.8) + 0.1*sin(41*t+2.1));
        fn0_provisional = fn0_base - dtau_ff*k_comp_true;   % fn0 decreases with +Delta_tau
        fn0_meas        = fn0_provisional + pseudo_noise;
        e_fn0           = P.fn0_d - fn0_meas;
        dtau_pi         = P.Kp_fn0*e_fn0*(-1) + P.Ki_fn0*ei_fn0*(-1);  % sign: +error -> reduce Delta_tau
    else
        dtau_pi = 0;
    end

    % ── TOTAL DIFFERENTIAL COMMAND (eq. u0_total) ────────────────────────
    dtau_d = clamp(dtau_ff + dtau_pi, -P.Delta_tau_max, P.Delta_tau_max);

    % Physical outcome uses the TRUE gain (eq. N0_affine): +Delta_tau lowers fn0
    fn0 = fn0_base - dtau_d*k_comp_true;
    fn0 = max(fn0, 0);

    % Total wheel torques: DIFFERENTIAL (antisymmetric) radial-force
    % superposition (OPPOSITE sign, via J^-1 = M/2, eq. actuation_jacobian_inv).
    % Common-mode sum tw1+tw2 = tau_sigma drives the ORBIT; the difference
    % tw1-tw2 = Delta_tau regulates fn0 without perturbing phi.
    tw1 = tw1_ff + 0.5*dtau_d;
    tw2 = tw2_ff - 0.5*dtau_d;
end

function v = trapz_vel(t, t_end, v_max)
    tr = 0.10*t_end;
    if     t < tr,          v = v_max * t/tr;
    elseif t < t_end-tr,    v = v_max;
    else,                   v = v_max * (t_end-t)/tr;
    end
end

function y = clamp(x,lo,hi), y = min(max(x,lo),hi); end

function [val,ist,dir] = evStop(~,x,phi_max)
    val = x(1)-phi_max; ist=1; dir=1;
end
