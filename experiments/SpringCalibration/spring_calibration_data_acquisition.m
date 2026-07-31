function spring_calibration_data_acquisition()
% ACQUIRE_SENSOR_SERIAL
% Receives the sensor readings transmitted by the Arduino via the serial port.
% Stores each acquired sample directly in a CSV file.
%
% Expected data format from the Arduino:
% DATA,time_ms,ADC,voltage,block

clc;
close all;

%% =========================================================
%  CONFIGURACIÓN
% ==========================================================

ARDUINO_PORT = '/dev/cu.usbserial-14240';
ARDUINO_BAUD = 115200;

%% =========================================================
%  CONFIGURATION
% ==========================================================

ARDUINO_PORT = '/dev/cu.usbserial-14240';
ARDUINO_BAUD = 115200;

DURACION_S = 40;             % Total experiment duration [s]
INTERVALO_PESO_S = 5;        % Weight change interval [s]

PESO_INICIAL_KG = 0.0;       % Applied weight during the first measurement block [kg]
INCREMENTO_PESO_KG = 0.1;    % Additional weight applied every 5 seconds [kg]

TIEMPO_MAXIMO_SIN_DATOS = 5; % Maximum time without receiving data before triggering disconnection protection [s]

%% =========================================================
%  GENERATE OUTPUT FILE NAME
% ==========================================================

fechaHora = datestr(now, 'yyyymmdd_HHMMSS');

nombreArchivo = sprintf( ...
    'datos_sensorA0_inicio200g-700g_%s.csv', fechaHora);

rutaArchivo = fullfile(pwd, nombreArchivo);

%% =========================================================
%  ABRIR PUERTO SERIAL
% ==========================================================

fprintf('Abriendo el puerto serial:\n%s\n\n', ARDUINO_PORT);

try
    arduinoSerial = serialport( ...
        ARDUINO_PORT, ...
        ARDUINO_BAUD, ...
        "Timeout", 2);
catch ME
    error([ ...
        'No se pudo abrir el puerto serial.\n' ...
        'Comprueba que el puerto sea correcto y que el Monitor Serial ' ...
        'de Arduino esté cerrado.\n\nError original:\n%s'], ...
        ME.message);
end

% Serial.println() envía CR/LF.
configureTerminator(arduinoSerial, "CR/LF");

% El Arduino Mega puede reiniciarse cuando MATLAB abre el puerto.
pause(2);

% Eliminar mensajes enviados durante el reinicio.
flush(arduinoSerial, "input");

%% =========================================================
%  ABRIR ARCHIVO CSV
% ==========================================================

archivoID = fopen(rutaArchivo, 'w');

if archivoID == -1
    clear arduinoSerial;
    error('No se pudo crear el archivo CSV.');
end

% Esta limpieza se ejecuta también si se detiene el programa con Ctrl+C.
limpieza = onCleanup( ...
    @() cerrarRecursos(arduinoSerial, archivoID));

% Encabezado del CSV.
fprintf(archivoID, ...
    ['Tiempo_s,ADC,Voltaje_V,Bloque,' ...
     'PesoProgramado_kg\n']);

%% =========================================================
%  PREPARAR GRÁFICA EN TIEMPO REAL
% ==========================================================

figura = figure( ...
    'Name', 'Adquisición del sensor', ...
    'NumberTitle', 'off');

lineaVoltaje = animatedline( ...
    'LineWidth', 1.4);

grid on;
xlabel('Tiempo (s)');
ylabel('Voltaje (V)');
title('Sensor conectado a A0');

%% =========================================================
%  INICIAR ARDUINO
% ==========================================================

fprintf('Enviando comando START al Arduino...\n');

writeline(arduinoSerial, "START");

ultimoBloque = -1;
numeroMuestras = 0;
ultimaRecepcion = tic;
experimentoTerminado = false;

fprintf('\nInicio de la adquisición.\n');
fprintf('Duración programada: %.1f segundos.\n', DURACION_S);
fprintf('Cambio de peso: cada %.1f segundos.\n\n', ...
    INTERVALO_PESO_S);

%% =========================================================
%  RECIBIR Y GUARDAR DATOS
% ==========================================================

while ~experimentoTerminado

    lineaRecibida = readline(arduinoSerial);

    % readline puede regresar vacío si vence el Timeout.
    if isempty(lineaRecibida)

        if toc(ultimaRecepcion) > TIEMPO_MAXIMO_SIN_DATOS
            error(['No se han recibido datos durante %.1f segundos. ' ...
                   'Comprueba la conexión USB.'], ...
                   TIEMPO_MAXIMO_SIN_DATOS);
        end

        continue;
    end

    lineaRecibida = strtrim(char(lineaRecibida));

    % Ignorar mensajes READY, STOPPED u otras líneas.
    if ~startsWith(lineaRecibida, 'DATA,')
        fprintf('Arduino: %s\n', lineaRecibida);
        continue;
    end

    % Interpretar:
    % DATA,tiempo_ms,ADC,voltaje,bloque
    valores = sscanf( ...
        lineaRecibida, ...
        'DATA,%f,%f,%f,%f');

    if numel(valores) ~= 4
        warning('Línea inválida ignorada: %s', lineaRecibida);
        continue;
    end

    ultimaRecepcion = tic;

    tiempoMs = valores(1);
    lecturaADC = round(valores(2));
    voltaje = valores(3);
    bloqueActual = round(valores(4));

    tiempoS = tiempoMs / 1000;

    % Peso que debería estar colocado durante este bloque.
    pesoProgramadoKg = ...
        PESO_INICIAL_KG + ...
        bloqueActual * INCREMENTO_PESO_KG;

    %% Aviso cada cinco segundos

    if bloqueActual ~= ultimoBloque

        ultimoBloque = bloqueActual;

        fprintf('\n====================================\n');

        if bloqueActual == 0
            fprintf('INICIO DEL EXPERIMENTO\n');
            fprintf('Coloque el peso inicial.\n');
        else
            fprintf('CAMBIO DE PESO\n');
            fprintf('Agregue %.3f kg.\n', ...
                INCREMENTO_PESO_KG);
        end

        fprintf('Tiempo: %.1f s\n', tiempoS);
        fprintf('Peso programado total: %.3f kg\n', ...
            pesoProgramadoKg);
        fprintf('Bloque: %d\n', bloqueActual);
        fprintf('====================================\n');

        beep;
    end

    %% Guardar inmediatamente en el CSV

    fprintf(archivoID, ...
        '%.3f,%d,%.6f,%d,%.6f\n', ...
        tiempoS, ...
        lecturaADC, ...
        voltaje, ...
        bloqueActual, ...
        pesoProgramadoKg);

    numeroMuestras = numeroMuestras + 1;

    %% Actualizar gráfica

    if isgraphics(lineaVoltaje)

        addpoints( ...
            lineaVoltaje, ...
            tiempoS, ...
            voltaje);

        limiteIzquierdo = max(0, tiempoS - 20);
        limiteDerecho = max(20, tiempoS);

        xlim([limiteIzquierdo, limiteDerecho]);

        title(sprintf( ...
            ['Sensor A2 | Tiempo: %.1f s | ' ...
             'Peso programado: %.3f kg'], ...
            tiempoS, pesoProgramadoKg));

        drawnow limitrate;
    end

    %% Terminar al alcanzar la duración indicada

    if tiempoS >= DURACION_S
        experimentoTerminado = true;
    end
end

%% =========================================================
%  CERRAR ARCHIVO Y PUERTO
% ==========================================================

% Ejecuta cerrarRecursos antes de continuar.
clear limpieza;

fprintf('\nAdquisición terminada correctamente.\n');
fprintf('Número de muestras: %d\n', numeroMuestras);
fprintf('Archivo guardado en:\n%s\n', rutaArchivo);

end


function cerrarRecursos(arduinoSerial, archivoID)
% Detiene el Arduino y cierra el CSV aunque ocurra un error.

try
    writeline(arduinoSerial, "STOP");
catch
    % El puerto podría haberse desconectado.
end

if archivoID ~= -1
    fclose(archivoID);
end

end