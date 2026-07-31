% =========================================================================
% Script for Generating Angle vs. Force Plot
% Data Averaged Across Three Experiments (Measurements Above 7 N Excluded)
% =========================================================================

clc; clear; close all;

% 1. Datos promediados calculados (Masas de 0.0 kg a 0.7 kg)
% F_n0 en Newtons
F_n0 = [0.000, 0.981, 1.962, 2.943, 3.924, 4.905, 5.886, 6.867];
% theta_0 en Grados (Inicia plano en 165.69)
theta_0 = [165.69, 165.69, 156.46, 142.40, 132.30, 127.81, 123.83, 120.24]-120.24;


% 2. Configuración y creación de la figura
% Tamaño optimizado para ajustarse a una columna de texto en un paper
fig = figure('Name', 'Fn_0 Rotational Spring ', 'Color', 'w', 'Position', [100, 100, 550, 400]);

% 3. Trazado de la curva (Estilo Clean / Escala de grises)
plot(F_n0, theta_0, '-bo', ...
    'LineWidth', 1.5, ...          % Grosor de la línea
    'MarkerSize', 4, ...           % Tamaño de los puntos
    'MarkerFaceColor', 'w', ...    % Interior del punto blanco
    'MarkerEdgeColor', 'b');       % Borde del punto negro

% 4. Formato de los ejes y cuadrícula
grid on;
ax = gca;
ax.GridLineStyle = '-';
ax.GridAlpha = 0.3;
ax.LineWidth = 0.6;
ax.FontSize = 12;
%ax.FontName = 'Times New Roman'; % Tipografía Serif estándar
ax.TickDir = 'in';               % Ticks hacia el interior (estilo IEEE/Elsevier)

% Limitar los ejes para enmarcar mejor la curva
xlim([0, 7]);
% ylim([-2, 45]);

% 5. Etiquetas con intérprete LaTeX
xlabel('$f$ [N]', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('$\theta_{0{_f}}$ $[\mathrm{deg}]$', 'Interpreter', 'latex', 'FontSize', 12);

