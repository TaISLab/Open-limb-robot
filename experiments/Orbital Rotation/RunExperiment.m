%% Robot Orbiting Test While Maintaining a Stable Grip on the Cylinder
% Robot: DYNAMIXEL XC330-T288-T + Arduino Mega + MPU6050 + sensores A0/A2
%
clear;
clc;

%% ========================================================
% PARÁMETROS MODIFICABLES ENTRE ENSAYOS
% =========================================================
MU_ESTIMADO         =0.5;     % CAMBIAR ESTE VALOR PARA CADA PRUEBA (0.2, 0.35, 0.5, 1.0)

% --- PARÁMETROS MECÁNICOS Y FÍSICOS ---
Kt_dxl              = 1150;    % Dynamixel torque constant [N·mm/A] (1.15 × 1000)
I_a_min             = 120;     % Baseline gripping current associated with tau_{a_min} [mA]
m                   = 0.280;   % Robot mass [kg]
g                   = 9.81;    % Gravitational acceleration [m/s^2]
r_c                 = 0.0560;  % Effective moment arm [m]
alpha_deg           = 30;      % Actuator force angle [deg]

% --- ORIGINAL MOTION PARAMETERS ---
EXTRA_CLOSING_DEG   = 40.0;    % Kept high to intentionally reach the current limit
TARGET_PULSES       = 500;     % Number of pulses required for continuous turning
STOP_ANGLE          = 180;     % Allows continuous rotation
GOAL_CURRENT        = 220;     % Approximate DXL current limit [mA]
TEST_DURATION       = 51;      % Maximum test duration for safety purposes [s]
RETURN_MARGIN       = 5.0;     % Angular return margin [deg]
CSV_NAME            = sprintf("Giro_Mu_%.2f.csv", MU_ESTIMADO);
CALIBRATION_TIME    = 5;       % Initial calibration time [s]
INITIAL_CLOSING     = 50;      % Maximum initial closing displacement [deg]
CONTACT_TIMEOUT     = 12;      % Maximum time allowed for contact detection [s]
SETTLING_TIME       = 1.0;     % Settling time after contact detection [s]

% Closing means DECREASING the DYNAMIXEL position.
CLOSING_SIGN        = -1;
PHI_SIGN            = +1;

CONTACT_CONFIRMATION_SAMPLES = 3;
ANGLE_CONFIRMATION_SAMPLES   = 3;
HOLD_TORQUE_AFTER_TEST       = true;
DISABLE_TORQUE_ON_ERROR      = true;

%% ========================================================
% DEFINICIÓN DE theta0 A PARTIR DE A0 Y A2
% =========================================================
A0_IDEAL_CONTACT_V       = 2.849;
A2_IDEAL_CONTACT_V       = 2.165;
THETA0_A0_WEIGHT         = 1.0; 
THETA0_A2_WEIGHT         = 0.0; 
THETA0_CONTACT_THRESHOLD = 0.45;
THETA0_FILTER_ALPHA      = 0.75;

%% ========================================================
% PUERTOS Y CONFIGURACIÓN DE COMUNICACIÓN
% =========================================================
ARDUINO_PORT         = "/dev/cu.usbserial-14240";
ARDUINO_BAUD         = 115200;
ARDUINO_TIMEOUT      = 0.10;
ARDUINO_STARTUP_WAIT = 3.0;
DXL_DEVICENAME       = '/dev/cu.usbserial-FTAKRP0V';
DXL_BAUDRATE         = 57600;
PROTOCOL_VERSION     = 2.0;
DXL_ID               = 1;
LIB_NAME             = 'libdxl_mac_c';
DXL_MAIN_HEADER      = 'dynamixel_sdk.h';

%% ========================================================
% TABLA DE CONTROL DYNAMIXEL XC330-T288-T
% =========================================================
ADDR_OPERATING_MODE         = 11;
ADDR_CURRENT_LIMIT          = 38;
ADDR_TORQUE_ENABLE          = 64;
ADDR_GOAL_CURRENT           = 102;
ADDR_GOAL_POSITION          = 116;
ADDR_PRESENT_CURRENT        = 126;
ADDR_PRESENT_POSITION       = 132;
CURRENT_BASED_POSITION_MODE = 5;
TORQUE_ENABLE               = 1;
TORQUE_DISABLE              = 0;
COMM_SUCCESS                = 0;
PULSES_PER_REVOLUTION       = 4096;
DEGREES_PER_PULSE           = 360 / PULSES_PER_REVOLUTION;
CURRENT_UNIT_MA             = 1.0;  
initialClosingPulses        = round(INITIAL_CLOSING / DEGREES_PER_PULSE);

%% ========================================================
% VARIABLES DE ESTADO INICIALES
% =========================================================
arduinoSerial         = [];
port_num              = [];
portOpened            = false;
torqueEnabled         = false;
normalFinish          = false;
errorOccurred         = false;
errorMessage          = "";
stopReason            = "NO_INICIADA";
AX                    = NaN; 
AY                    = NaN; 
AZ                    = NaN; 
A0                    = NaN; 
A2                    = NaN;
phi0                  = NaN; 
A0_ref                = NaN; 
A2_ref                = NaN;
theta0DenA0           = NaN; 
theta0DenA2           = NaN;

% Contenedores de datos
timeData              = []; 
phiRawData            = []; 
phiRelativeData       = [];
axData                = []; 
ayData                = []; 
azData                = [];
a0Data                = []; 
a2Data                = []; 
theta0Data            = []; 
theta0FilteredData    = [];
dxlCurrentData        = []; 
dxlCurrentAbsData     = [];
dxlPositionPulseData  = []; 
dxlPositionDegreeData = []; 
dxlGoalPositionDegreeData = [];
pwmData               = []; 
arduinoAgeData        = []; 
tauDinamicoData       = []; 
targetCurrentData     = [];
metadata              = struct();

%% ========================================================
% CERRAR CONEXIONES SERIALPORT PREVIAS DE MATLAB
% =========================================================
previousConnections = serialportfind;
if ~isempty(previousConnections)
    delete(previousConnections);
end

%% ========================================================
% PROGRAMA PRINCIPAL
% =========================================================
try
    %% 1. CONEXIÓN CON ARDUINO
    fprintf('\n====================================================\n');
    fprintf('PRUEBA DEL ROBOT\n');
    fprintf('====================================================\n');
    arduinoSerial = serialport(ARDUINO_PORT, ARDUINO_BAUD, "Timeout", ARDUINO_TIMEOUT);
    configureTerminator(arduinoSerial, "LF");
    fprintf('Esperando inicialización del Arduino...\n');
    pause(ARDUINO_STARTUP_WAIT);
    flush(arduinoSerial);
    writeline(arduinoSerial, "STOP");
    pause(0.1);
    fprintf('Arduino conectado: %s a %d bps.\n', char(ARDUINO_PORT), ARDUINO_BAUD);

    %% 2. CARGAR Y CONECTAR DYNAMIXEL SDK
    if ~libisloaded(LIB_NAME)
        loadlibrary(LIB_NAME, DXL_MAIN_HEADER, 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    end
    port_num = portHandler(DXL_DEVICENAME);
    packetHandler();
    if ~openPort(port_num)
        error('No fue posible abrir el puerto U2D2 %s.', DXL_DEVICENAME);
    end
    portOpened = true;
    if ~setBaudRate(port_num, DXL_BAUDRATE)
        error('No fue posible configurar el baudrate del DYNAMIXEL.');
    end
    fprintf('DYNAMIXEL conectado: %s a %d bps.\n', DXL_DEVICENAME, DXL_BAUDRATE);

    %% 3. CONFIGURAR CURRENT-BASED POSITION CONTROL
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'desactivar el torque');
    torqueEnabled = false;
    
    currentMode = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'leer Operating Mode');
    if double(currentMode) ~= CURRENT_BASED_POSITION_MODE
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE);
        assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'configurar Current-based Position Control');
    end
    
    currentLimitRaw = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_CURRENT_LIMIT);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'leer Current Limit');
    currentLimit_mA = double(currentLimitRaw) * CURRENT_UNIT_MA;
    
    if GOAL_CURRENT > currentLimit_mA
        warning('GOAL_CURRENT=%.1f mA supera Current Limit=%.1f mA. Se limitará al valor permitido.', GOAL_CURRENT, currentLimit_mA);
        goalCurrentCommand_mA = floor(currentLimit_mA);
    else
        goalCurrentCommand_mA = GOAL_CURRENT;
    end
    goalCurrentRaw = int16(round(goalCurrentCommand_mA / CURRENT_UNIT_MA));
    
    presentPositionRaw = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'leer la posición inicial');
    initialHoldPosition = typecast(uint32(presentPositionRaw), 'int32');
    
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(initialHoldPosition, 'uint32'));
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'fijar la posición inicial');
    
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, typecast(goalCurrentRaw, 'uint16'));
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'configurar Goal Current');
    
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'activar el torque');
    torqueEnabled = true;
    
    fprintf('Modo Current-based Position Control activado.\n');
    fprintf('Goal Current configurado: %.0f mA.\n', goalCurrentCommand_mA);

    %% 4. CALIBRACIÓN INICIAL DE MPU, A0 Y A2
    fprintf('\nCalibración durante %.1f s. Mantén el robot quieto.\n', CALIBRATION_TIME);
    flush(arduinoSerial);
    calibrationPhi = []; calibrationA0 = []; calibrationA2 = [];
    calibrationTimer = tic;
    while toc(calibrationTimer) < CALIBRATION_TIME
        [updated, AXnew, AYnew, AZnew, A0new, A2new] = readLatestArduinoData(arduinoSerial);
        if updated
            AX = AXnew; AY = AYnew; AZ = AZnew; A0 = A0new; A2 = A2new;
            phiCurrent = PHI_SIGN * atan2d(AX, AZ);
            calibrationPhi(end + 1, 1) = phiCurrent; %#ok<SAGROW>
            calibrationA0(end + 1, 1) = A0; %#ok<SAGROW>
            calibrationA2(end + 1, 1) = A2; %#ok<SAGROW>
        end
        pause(0.005);
    end
    if numel(calibrationPhi) < 20
        error('Se recibieron solamente %d muestras válidas durante la calibración.', numel(calibrationPhi));
    end
    phi0 = atan2d(mean(sind(calibrationPhi), 'omitnan'), mean(cosd(calibrationPhi), 'omitnan'));
    A0_ref = mean(calibrationA0, 'omitnan');
    A2_ref = mean(calibrationA2, 'omitnan');
    theta0DenA0 = A0_ref - A0_IDEAL_CONTACT_V;
    theta0DenA2 = A2_IDEAL_CONTACT_V - A2_ref;
    if theta0DenA0 <= 0.05
        theta0DenA0 = 0.50;
    end
    if theta0DenA2 <= 0.05
        theta0DenA2 = 0.60;
    end
    fprintf('\nReferencias calculadas:\n  phi0 = %.3f deg\n  A0_ref = %.4f V\n  A2_ref = %.4f V\n', phi0, A0_ref, A2_ref);

    %% 5. LEER POSICIÓN INICIAL DESPUÉS DE CALIBRAR
    presentPositionRaw = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'leer posición antes del cierre');
    initialPosition = typecast(uint32(presentPositionRaw), 'int32');
    initialPosition_deg = double(initialPosition) * DEGREES_PER_PULSE;
    closingTargetPosition = int32(double(initialPosition) + CLOSING_SIGN * initialClosingPulses);
    fprintf('\nPosición inicial DXL: %d pulsos, %.2f deg.\n', initialPosition, initialPosition_deg);

    %% 6. INICIAR CIERRE Y DETECTAR CONTACTO CON theta0
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(closingTargetPosition, 'uint32'));
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'iniciar el cierre');
    fprintf('\nCerrando el DYNAMIXEL y buscando contacto...\n');
    contactDetected = false; contactCounter = 0;
    filteredTheta0 = NaN; contactPosition = initialPosition;
    closureTimer = tic;
    while toc(closureTimer) < CONTACT_TIMEOUT
        [updated, AXnew, AYnew, AZnew, A0new, A2new] = readLatestArduinoData(arduinoSerial);
        if updated
            AX = AXnew; AY = AYnew; AZ = AZnew; A0 = A0new; A2 = A2new;
            theta0Raw = computeTheta0(A0, A2, A0_ref, A2_ref, theta0DenA0, theta0DenA2, THETA0_A0_WEIGHT, THETA0_A2_WEIGHT);
            if isnan(filteredTheta0)
                filteredTheta0 = theta0Raw;
            else
                filteredTheta0 = THETA0_FILTER_ALPHA * filteredTheta0 + (1 - THETA0_FILTER_ALPHA) * theta0Raw;
            end
            if filteredTheta0 >= THETA0_CONTACT_THRESHOLD
                contactCounter = contactCounter + 1;
            else
                contactCounter = 0;
            end
            if contactCounter >= CONTACT_CONFIRMATION_SAMPLES
                presentPositionRaw = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
                assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'leer posición al detectar contacto');
                contactPosition = typecast(uint32(presentPositionRaw), 'int32');
                contactDetected = true;
                break;
            end
        end
        presentPositionRaw = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        presentPositionDuringClosure = typecast(uint32(presentPositionRaw), 'int32');
        positionErrorPulses = abs(double(closingTargetPosition) - double(presentPositionDuringClosure));
        if positionErrorPulses <= round(1.0 / DEGREES_PER_PULSE) && toc(closureTimer) > 1.0
            error('El DYNAMIXEL alcanzó el cierre máximo sin detectar contacto.');
        end
        pause(0.015);
    end
    if ~contactDetected
        error('No se detectó contacto.');
    end
    fprintf('CONTACTO detectado: theta0 filtrado = %.3f.\n', filteredTheta0);

    %% 7. MANTENER LA POSICIÓN EN LA QUE SE DETECTÓ EL CONTACTO
    extraPulses = round(EXTRA_CLOSING_DEG / DEGREES_PER_PULSE);
    finalGoalPosition = contactPosition + (CLOSING_SIGN * extraPulses);
    finalGoalPosition_deg = double(finalGoalPosition) * DEGREES_PER_PULSE;
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(finalGoalPosition, 'uint32'));
    assertDxlCommunication(port_num, PROTOCOL_VERSION, COMM_SUCCESS, 'mantener la posición de contacto');
    fprintf('Posición de contacto mantenida. Estabilizando el agarre...\n');
    pause(SETTLING_TIME);

    %% 8. INICIAR MOVIMIENTO DE LOS MOTORES DC
    flush(arduinoSerial);
    startCommand = sprintf('START,%d', TARGET_PULSES);
    writeline(arduinoSerial, startCommand);
    fprintf('\nMovimiento iniciado con comando: %s\n', startCommand);
    fprintf('La prueba se detendrá al alcanzar %.1f deg o %.1f s.\n\n', STOP_ANGLE, TEST_DURATION);

    %% 9. ADQUISICIÓN DURANTE EL GIRO (Optimizado y Robusto)
    testTimer                  = tic;
    arduinoSampleTime          = 0;
    filteredTheta0             = NaN;
    angleCounter               = 0;
    sampleIndex                = 0;
    lastConsolePrint           = -inf;
    PWM_actual                 = 0;
    
    % Variables de estado para el Unwrap robusto
    primera_lectura_unwrap     = true;
    phiRaw_anterior            = 0;
    phi_continuo               = 0;
    phiRaw                     = 0;
    
    currentPhase               = 1; % 1 = Fase de Ida, 2 = Fase de Retorno
    
    while true
        currentTime = toc(testTimer);
        
        % Leer la muestra Arduino más reciente disponible.
        [updated, AXnew, AYnew, AZnew, A0new, A2new, PWMnew, PHInew] = readLatestArduinoData(arduinoSerial);

        if updated
            AX = AXnew; AY = AYnew; AZ = AZnew;
            A0 = A0new; A2 = A2new;
            PWM_actual = PWMnew;

            phiRaw = PHInew;
            arduinoSampleTime   = currentTime;
        end

        if any(isnan([AX, AY, AZ, A0, A2, phiRaw]))
            pause(0.005);
            continue;
        end
        
        % CÁLCULO DEL ÁNGULO RELATIVO
        phiRelative = phiRaw - phi0;
        
        % ---------------------------------------------------------
        % CÁLCULO DINÁMICO DE TORQUE (ECUACIÓN DE FRICCIÓN)
        % ---------------------------------------------------------
        phi_rad               = deg2rad(phiRelative);
        tau_dinamico_Nm       = (r_c / (MU_ESTIMADO * cosd(alpha_deg))) * m * g * abs(sin(phi_rad));
        tau_dinamico_Nmm      = (tau_dinamico_Nm * 1000);
        current_dinamica_mA   = (tau_dinamico_Nmm / Kt_dxl) * 1000;
        targetCurrent_mA      = I_a_min + current_dinamica_mA;
        if targetCurrent_mA > 500
            targetCurrent_mA  = 500;
        end
        goalCurrentRaw        = int16(round(targetCurrent_mA / CURRENT_UNIT_MA));
        write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, typecast(goalCurrentRaw, 'uint16'));
        
        % --------------------------------------------------
        theta0Raw = computeTheta0(A0, A2, A0_ref, A2_ref, theta0DenA0, theta0DenA2, THETA0_A0_WEIGHT, THETA0_A2_WEIGHT);
        if isnan(filteredTheta0)
            filteredTheta0 = theta0Raw;
        else
            filteredTheta0 = THETA0_FILTER_ALPHA * filteredTheta0 + (1 - THETA0_FILTER_ALPHA) * theta0Raw;
        end
        
        % Lecturas al Dynamixel
        presentCurrentRawUnsigned = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        presentCurrent_mA         = double(typecast(uint16(presentCurrentRawUnsigned), 'int16')) * CURRENT_UNIT_MA;
        
        presentPositionRawUnsigned = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        presentPosition            = typecast(uint32(presentPositionRawUnsigned), 'int32');
        presentPosition_deg        = double(presentPosition) * DEGREES_PER_PULSE;
        
        sampleIndex                             = sampleIndex + 1;
        timeData(sampleIndex, 1)                = currentTime;
        phiRawData(sampleIndex, 1)              = phiRaw;
        phiRelativeData(sampleIndex, 1)         = phiRelative;
        axData(sampleIndex, 1)                  = AX;
        ayData(sampleIndex, 1)                  = AY;
        azData(sampleIndex, 1)                  = AZ;
        a0Data(sampleIndex, 1)                  = A0;
        a2Data(sampleIndex, 1)                  = A2;
        theta0Data(sampleIndex, 1)              = theta0Raw;
        theta0FilteredData(sampleIndex, 1)      = filteredTheta0;
        dxlCurrentData(sampleIndex, 1)          = presentCurrent_mA;
        dxlCurrentAbsData(sampleIndex, 1)       = abs(presentCurrent_mA);
        dxlPositionPulseData(sampleIndex, 1)    = double(presentPosition);
        dxlPositionDegreeData(sampleIndex, 1)   = presentPosition_deg;
        dxlGoalPositionDegreeData(sampleIndex, 1) = finalGoalPosition_deg;
        pwmData(sampleIndex, 1)                 = PWM_actual; 
        arduinoAgeData(sampleIndex, 1)          = max(0, currentTime - arduinoSampleTime);
        tauDinamicoData(sampleIndex, 1)         = tau_dinamico_Nmm;
        targetCurrentData(sampleIndex, 1)       = targetCurrent_mA;
        
        if currentTime - lastConsolePrint >= 0.2
            fprintf(['t:%6.2f s | phi:%7.2f deg | A0:%.3f V | A2:%.3f V | ' ...
                'theta0:%5.2f | I:%7.1f mA | DXL:%8.2f deg\n'], ...
                currentTime, phiRelative, A0, A2, filteredTheta0, ...
                presentCurrent_mA, presentPosition_deg);
            lastConsolePrint = currentTime;
        end
        
        % ----------------------------------------------------
        % GESTIÓN DE LÍMITES
        % ----------------------------------------------------
        if currentPhase == 1
            angleReached = abs(phiRelative) >= STOP_ANGLE;
        else
            angleReached = abs(phiRelative) <= RETURN_MARGIN;
        end
        
        if angleReached
            angleCounter = angleCounter + 1;
        else
            angleCounter = 0;
        end
        
        if angleCounter >= ANGLE_CONFIRMATION_SAMPLES
            if currentPhase == 1
                fprintf('\n¡Límite de %d grados alcanzado!\n', STOP_ANGLE);
                fprintf('Iniciando el retorno a 0 grados...\n');
                currentPhase = 2; 
                angleCounter = 0; 
                startCommandRev = sprintf('START,%d', -TARGET_PULSES);
                writeline(arduinoSerial, startCommandRev);
            else
                fprintf('\n¡Posición inicial de 0 grados alcanzada!\n');
                fprintf('Iniciando un nuevo ciclo de ida...\n');
                currentPhase = 1;
                angleCounter = 0;
                startCommandFwd = sprintf('START,%d', TARGET_PULSES);
                writeline(arduinoSerial, startCommandFwd);
            end
        end
        
        if currentTime >= TEST_DURATION
            stopReason = "TIEMPO_MAXIMO";
            break;
        end
        
        pause(0.002);
    end

    normalFinish = true;

catch ME
    errorOccurred = true;
    errorMessage  = string(ME.message);
    stopReason    = "ERROR";
    fprintf('\nERROR: %s\n', ME.message);
end

%% ========================================================
% DETENCIÓN SEGURA DE MOTORES Y GESTIÓN DEL TORQUE
% =========================================================
if ~isempty(arduinoSerial)
    try
        writeline(arduinoSerial, "STOP");
        pause(0.05);
    catch
    end
end

if torqueEnabled
    mustDisableTorque = (~normalFinish && DISABLE_TORQUE_ON_ERROR) || (normalFinish && ~HOLD_TORQUE_AFTER_TEST);
    if mustDisableTorque && portOpened
        try
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
            torqueEnabled = false;
            fprintf('Torque DYNAMIXEL desactivado.\n');
        catch
        end
    elseif normalFinish && HOLD_TORQUE_AFTER_TEST
        fprintf('El DYNAMIXEL permanece cerrado con torque habilitado.\n');
    end
end

%% ========================================================
% GUARDAR CSV Y MAT
% =========================================================
if ~isempty(timeData)
    Test_ID     = repmat(string(erase(CSV_NAME, ".csv")), numel(timeData), 1);
    Stop_Reason = repmat(stopReason, numel(timeData), 1);
    
    testTable = table(timeData, phiRelativeData, phiRawData, axData, ayData, azData, ...
        a0Data, a2Data, theta0Data, theta0FilteredData, dxlCurrentData, dxlCurrentAbsData, ...
        dxlPositionPulseData, dxlPositionDegreeData, dxlGoalPositionDegreeData, pwmData, ...
        arduinoAgeData, tauDinamicoData, targetCurrentData, Test_ID, Stop_Reason, ...
        'VariableNames', {'Tiempo_s', 'Phi_rel_deg', 'Phi_raw_deg', 'AX_g', 'AY_g', 'AZ_g', ...
        'A0_V', 'A2_V', 'Theta0_raw', 'Theta0_filtrado', 'DXL_Corriente_mA', 'DXL_Corriente_abs_mA', ...
        'DXL_Posicion_pulsos', 'DXL_Posicion_deg', 'DXL_Goal_Position_deg', 'Pulsos_Objetivo', ...
        'Edad_dato_Arduino_s', 'Tau_calculado_Nmm', 'Corriente_calculada_mA', 'Test_ID', 'Motivo_detencion'});
    
    writetable(testTable, CSV_NAME);
    fprintf('CSV guardado: %s\n', char(CSV_NAME));
    
    metadata.TARGET_PULSES              = TARGET_PULSES;
    metadata.STOP_ANGLE_deg             = STOP_ANGLE;
    metadata.GOAL_CURRENT_requested_mA  = GOAL_CURRENT;
    metadata.TEST_DURATION_s            = TEST_DURATION;
    metadata.CALIBRATION_TIME_s         = CALIBRATION_TIME;
    metadata.phi0_deg                   = phi0;
    metadata.A0_ref_V                   = A0_ref;
    metadata.A2_ref_V                   = A2_ref;
    metadata.stopReason                 = stopReason;
    metadata.normalFinish               = normalFinish;
    metadata.errorOccurred              = errorOccurred;
    metadata.errorMessage               = errorMessage;
    
    matName = replace(CSV_NAME, ".csv", ".mat");
    save(char(matName), 'testTable', 'metadata');
    fprintf('MAT guardado: %s\n', char(matName));
else
    warning('No existen muestras de giro para guardar.');
end

%% ========================================================
% CERRAR COMUNICACIONES
% =========================================================
if portOpened
    try
        closePort(port_num);
    catch
    end
end

if ~isempty(arduinoSerial)
    try
        delete(arduinoSerial);
    catch
    end
end

if libisloaded(LIB_NAME)
    try
        unloadlibrary(LIB_NAME);
    catch
    end
end

fprintf('\nFin de la prueba.\n');

if errorOccurred
    error('La prueba terminó con error: %s', char(errorMessage));
end

%% ========================================================
% FUNCIONES LOCALES
% =========================================================
function assertDxlCommunication(port_num, protocolVersion, commSuccess, actionText)
    commResult  = getLastTxRxResult(port_num, protocolVersion);
    packetError = getLastRxPacketError(port_num, protocolVersion);
    if commResult ~= commSuccess
        error('Fallo de comunicación DYNAMIXEL al %s. Código: %d.', actionText, commResult);
    end
    if packetError ~= 0
        error('Error de paquete DYNAMIXEL al %s. Código: %d.', actionText, packetError);
    end
end

function [valid, AX, AY, AZ, A0, A2, PWM_real, PHI_clean] = parseArduinoLine(line)
    numberPattern = '([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)';
    pattern = ['AX_g:\s*' numberPattern '.*?' 'AY_g:\s*' numberPattern '.*?' 'AZ_g:\s*' numberPattern '.*?' 'A0_V:\s*' numberPattern '.*?' 'A2_V:\s*' numberPattern '.*?' 'PWM:\s*' numberPattern '.*?' 'PHI_deg:\s*' numberPattern];
    tokens = regexp(char(line), pattern, 'tokens', 'once');
    if isempty(tokens)
        valid = false;
        AX = NaN; AY = NaN; AZ = NaN; A0 = NaN; A2 = NaN; PWM_real = NaN; PHI_clean = NaN;
        return;
    end
    values = cellfun(@str2double, tokens);
    valid = all(isfinite(values));
    AX = values(1); AY = values(2); AZ = values(3);
    A0 = values(4); A2 = values(5); PWM_real = values(6); PHI_clean = values(7);
end

function [updated, AX, AY, AZ, A0, A2, PWM_real, PHI_clean] = readLatestArduinoData(serialObject)
    updated = false;
    AX = NaN; AY = NaN; AZ = NaN; A0 = NaN; A2 = NaN; PWM_real = NaN; PHI_clean = NaN;
    while serialObject.NumBytesAvailable > 0
        line = strtrim(readline(serialObject));
        [valid, AXtmp, AYtmp, AZtmp, A0tmp, A2tmp, PWMtmp, PHItmp] = parseArduinoLine(line);
        if valid
            AX = AXtmp; AY = AYtmp; AZ = AZtmp;
            A0 = A0tmp; A2 = A2tmp; PWM_real = PWMtmp; PHI_clean = PHItmp;
            updated = true;
        end
    end
end

function theta0 = computeTheta0(A0, A2, A0ref, A2ref, denA0, denA2, weightA0, weightA2)
    thetaA0 = (A0ref - A0) / denA0;
    thetaA2 = (A2 - A2ref) / denA2;
    theta0  = weightA0 * thetaA0 + weightA2 * thetaA2;
end