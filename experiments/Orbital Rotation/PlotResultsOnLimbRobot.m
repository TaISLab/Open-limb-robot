%% SCRIPT FOR PLOT RESULTS
clear; clc; close all;

%% 
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

%% 
I_stall   =0.6;
I_no_load =0.06;
Kt = 0.118; 
dxlTorque_Nm_min = 138;

xlimi = 51;

% 1. Experimental Files
archivos = {
    'Giro_Mu_0.50.csv',
    'Giro_Mu_1.00.csv',
    'Giro_Mu_1.70.csv',
    'Giro_Mu_2.20.csv'
};

% 2. Colors
colores = {'b','r', 'g','k'}; 
leyendas = {'$\mu = 0.50$','$\mu = 1.00$', '$\mu = 1.70$', '$\mu = 2.20$'};

figure('Name', 'PLOT RESULTS', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% 3. Plotting
for i = 1:length(archivos)
    
    % Read current CSV 
    data = readtable(archivos{i});
    
    t = data.Tiempo_s;
    tau_a = data.Tau_calculado_Nmm;
    
    % ---------------------------------------------------------
    % SUBPLOT 1: Torque (\tau_w)
    % ---------------------------------------------------------
    subplot(3, 1, 1);
    hold on;

    pwmNormalizado = data.Pulsos_Objetivo/ 255;
    tau_w = data.Pulsos_Objetivo*Kt;
  
    plot(t, tau_w, 'Color', colores{i}, 'LineWidth', 1.5);
    xlim([0, xlimi]);
    ylabel('\tau_w [Nmm]', 'Interpreter', 'tex');
    xlabel('Time [s]', 'Interpreter', 'tex');
    grid on;
    
    % ---------------------------------------------------------
    % SUBPLOT 2: Torque Dynamixel (T_a)
    % ---------------------------------------------------------
    subplot(3, 1, 2);
    hold on; 
    plot(t, data.Tau_calculado_Nmm+dxlTorque_Nm_min, 'Color', colores{i}, 'LineWidth', 1.5);
    xlim([0, xlimi])
    ylim([70, 550])
    % La referencia de phi es vertical, por eso se usa yline.
    yline(dxlTorque_Nm_min, 'k--', '$\tau_{a_{min}}$','Interpreter', 'latex', ...
    'HandleVisibility', 'off', ...
    'LineWidth', 0.8, ...
    'FontSize', 12, ...
    'LabelVerticalAlignment','bottom',...
    'LabelHorizontalAlignment', 'left');
    xlabel('Time [s]', 'Interpreter', 'tex');
    ylabel('\tau_a [Nmm]', 'Interpreter', 'tex');
    grid on;
    
    % ---------------------------------------------------------
    % SUBPLOT 3: angle(\phi)
    % ---------------------------------------------------------
    subplot(3, 1, 3);
    hold on; 
    plot(t, data.Phi_rel_deg, 'Color', colores{i}, 'LineWidth', 1.5);
    xlim([0, xlimi])
    ylim([-30, 220])
    yline(180, 'k--','$\phi_{B}$','Interpreter', 'latex', ...
        'HandleVisibility', 'off', ...
        'LineWidth', 0.8, ...
        'FontSize', 12, ...
        'LabelHorizontalAlignment', 'left');
    yline(0, 'k--','$\phi_{A}$','Interpreter', 'latex', ...
        'HandleVisibility', 'off', ...
        'LineWidth', 0.8, ...
        'FontSize', 12, ...
        'LabelHorizontalAlignment', 'left','LabelVerticalAlignment','bottom');
    xlabel('Time [s]', 'Interpreter', 'tex');
    ylabel('\phi [deg]', 'Interpreter', 'tex');
    grid on;

end

subplot(3,1,1); legend(leyendas, 'Location', 'best');
subplot(3,1,2); legend(leyendas, 'Location', 'best');
subplot(3,1,3); legend(leyendas, 'Location', 'best');