#include <Wire.h>

// ========================================================
// PINES
// ========================================================
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 11;
const int IN4 = 10;

const int ENC1_A = 2;
const int ENC1_B = 4;
const int ENC2_A = 3;
const int ENC2_B = 5;

// ========================================================
// VARIABLES GLOBALES: CONTROL EN BUCLE ABIERTO Y ESTADO
// ========================================================
volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

uint8_t mpuAddress = 0;

bool motorsActive = false;

// Se conserva para mantener la compatibilidad con el comando START.
// En bucle abierto, únicamente se utiliza su signo para definir el sentido.
long targetPos = 0;

// 1 = Ida, -1 = Retorno
int currentDirection = 1;

// PWM constante aplicado durante las pruebas en bucle abierto
const int FIXED_PWM = 255;

float phi_complementario = 0.0;

unsigned long lastTime = 0;

// Intervalo de actualización del accionamiento y la telemetría [ms]
const unsigned long CONTROL_INTERVAL = 10;

// ========================================================
// FUNCIONES DE COMUNICACIÓN I2C
// ========================================================
bool escribirRegistro(
  uint8_t direccion,
  uint8_t registro,
  uint8_t valor
) {
  Wire.beginTransmission(direccion);
  Wire.write(registro);
  Wire.write(valor);

  return Wire.endTransmission() == 0;
}

bool leerRegistro(
  uint8_t direccion,
  uint8_t registro,
  uint8_t &valor
) {
  Wire.beginTransmission(direccion);
  Wire.write(registro);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(direccion, (uint8_t)1) != 1) {
    return false;
  }

  valor = Wire.read();
  return true;
}

// ========================================================
// INTERRUPCIONES DE LOS ENCODERS
// ========================================================
void readEncoder1() {
  if (digitalRead(ENC1_B) == HIGH) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void readEncoder2() {
  if (digitalRead(ENC2_B) == HIGH) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
}

// ========================================================
// CONFIGURACIÓN INICIAL
// ========================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopAllMotors();

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC1_A),
    readEncoder1,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC2_A),
    readEncoder2,
    RISING
  );

  Wire.begin();
  Wire.setClock(100000);

  if (!detectarMPU6050()) {
    // El programa permanece bloqueado si no se detecta el sensor.
    while (true) {
      delay(1000);
    }
  }

  escribirRegistro(
    mpuAddress,
    0x6B,
    0x00
  );  // Activar el MPU6050

  escribirRegistro(
    mpuAddress,
    0x1C,
    0x00
  );  // Configurar el acelerómetro en ±2 g
}

// ========================================================
// BUCLE PRINCIPAL
// ========================================================
void loop() {
  unsigned long currentTime = millis();

  checkSerialCommands();

  if (currentTime - lastTime >= CONTROL_INTERVAL) {
    lastTime = currentTime;

    int appliedPWM = 0;

    if (motorsActive) {
      // Funcionamiento en bucle abierto:
      // el PWM no depende de los encoders ni de un error de posición.
      appliedPWM = currentDirection * FIXED_PWM;

      setMotorPWM(1, appliedPWM);
      setMotorPWM(2, appliedPWM);
    } else {
      stopAllMotors();
    }

    // Se transmite el PWM realmente aplicado.
    sendTelemetry(appliedPWM);
  }
}

// ========================================================
// CONTROL DE MOTORES: PWM FIJO CON SIGNO
// ========================================================
void setMotorPWM(
  int motorNum,
  float controlSignal
) {
  int pwm = constrain(
    (int)controlSignal,
    -255,
    255
  );

  if (abs(pwm) < 25) {
    pwm = 0;
  }

  if (motorNum == 1) {
    // Desactivar inicialmente ambas entradas del motor 1.
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    // Aplicar el sentido de giro correspondiente.
    if (pwm >= 0) {
      analogWrite(IN1, pwm);
    } else {
      analogWrite(IN2, abs(pwm));
    }
  } else {
    // Desactivar inicialmente ambas entradas del motor 2.
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // Aplicar el sentido de giro correspondiente.
    if (pwm >= 0) {
      analogWrite(IN3, pwm);
    } else {
      analogWrite(IN4, abs(pwm));
    }
  }
}

void stopAllMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ========================================================
// TELEMETRÍA
// FILTRO COMPLEMENTARIO CONTINUO CON UNWRAPPING ANGULAR
// ========================================================
void sendTelemetry(int appliedPWM) {
  int16_t axRaw;
  int16_t ayRaw;
  int16_t azRaw;
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;

  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;

  float dt = CONTROL_INTERVAL / 1000.0;

  if (
    leerMPU(
      axRaw,
      ayRaw,
      azRaw,
      gxRaw,
      gyRaw,
      gzRaw
    )
  ) {
    ax = axRaw / 16384.0;
    ay = ayRaw / 16384.0;
    az = azRaw / 16384.0;

    // 1. Ángulo calculado a partir del acelerómetro [-180, 180].
    float accel_angle =
      atan2(ax, az) * 180.0 / PI;

    // 2. Velocidad angular del giroscopio [deg/s].
    float gyro_rate =
      gyRaw / 131.0;

    // 3. Predicción angular basada en la integración del giroscopio.
    float phi_predicho =
      phi_complementario + (gyro_rate * dt);

    // 4. Diferencia angular limitada al intervalo [-180, 180].
    float diff =
      accel_angle - phi_predicho;

    diff = fmod(
      diff + 180.0,
      360.0
    );

    if (diff < 0) {
      diff += 360.0;
    }

    diff -= 180.0;

    // 5. Actualización continua del filtro complementario.
    phi_complementario =
      phi_predicho + (0.02 * diff);
  }

  // Lectura de las entradas analógicas.
  float a0_real =
    analogRead(A0) * (5.0 / 1023.0);

  float a2_real =
    analogRead(A2) * (5.0 / 1023.0);

  int pwmRegistro = constrain(
    appliedPWM,
    -255,
    255
  );

  // Telemetría enviada a MATLAB.
  Serial.print("AX_g:");
  Serial.print(ax, 4);
  Serial.print(" ");

  Serial.print("AY_g:");
  Serial.print(ay, 4);
  Serial.print(" ");

  Serial.print("AZ_g:");
  Serial.print(az, 4);
  Serial.print(" ");

  Serial.print("A0_V:");
  Serial.print(a0_real, 3);
  Serial.print(" ");

  Serial.print("A2_V:");
  Serial.print(a2_real, 3);
  Serial.print(" ");

  Serial.print("PWM:");
  Serial.print(pwmRegistro);
  Serial.print(" ");

  Serial.print("PHI_deg:");
  Serial.println(phi_complementario, 3);
}

// ========================================================
// COMANDOS RECIBIDOS DESDE MATLAB
// ========================================================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd =
      Serial.readStringUntil('\n');

    // Elimina espacios y posibles caracteres '\r'.
    cmd.trim();

    if (cmd.startsWith("START,")) {
      targetPos =
        cmd.substring(6).toInt();

      // En bucle abierto, el signo del valor recibido
      // determina exclusivamente el sentido de giro.
      if (targetPos > 0) {
        currentDirection = 1;
      } else if (targetPos < 0) {
        currentDirection = -1;
      } else {
        motorsActive = false;
        stopAllMotors();
        return;
      }

      // Reinicio de los encoders al comenzar una nueva fase.
      encoder1Pos = 0;
      encoder2Pos = 0;

      motorsActive = true;
    } else if (cmd.equals("STOP")) {
      motorsActive = false;
      stopAllMotors();
    }
  }
}

// ========================================================
// DETECCIÓN DEL MPU6050
// ========================================================
bool detectarMPU6050() {
  const uint8_t direcciones[] = {
    0x68,
    0x69
  };

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t direccion =
      direcciones[i];

    Wire.beginTransmission(direccion);

    if (Wire.endTransmission() == 0) {
      uint8_t whoAmI;

      if (
        leerRegistro(
          direccion,
          0x75,
          whoAmI
        )
      ) {
        Serial.print(
          "Dispositivo encontrado en 0x"
        );
        Serial.println(
          direccion,
          HEX
        );

        Serial.print(
          "Registro WHO_AM_I: 0x"
        );
        Serial.println(
          whoAmI,
          HEX
        );

        if (
          whoAmI == 0x68 ||
          whoAmI == 0x69
        ) {
          mpuAddress = direccion;
          return true;
        }
      }
    }
  }

  return false;
}

// ========================================================
// LECTURA DEL MPU6050
// ========================================================
bool leerMPU(
  int16_t &ax,
  int16_t &ay,
  int16_t &az,
  int16_t &gx,
  int16_t &gy,
  int16_t &gz
) {
  // Comenzar en el registro ACCEL_XOUT_H.
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Solicitar 14 bytes:
  // 6 del acelerómetro, 2 de temperatura y 6 del giroscopio.
  uint8_t recibidos =
    Wire.requestFrom(
      mpuAddress,
      (uint8_t)14
    );

  if (recibidos != 14) {
    return false;
  }

  // Combinar los bytes alto y bajo del acelerómetro.
  ax = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  ay = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  az = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  // Descartar los dos bytes correspondientes a la temperatura.
  Wire.read();
  Wire.read();

  // Combinar los bytes alto y bajo del giroscopio.
  gx = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  gy = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  gz = (int16_t)(
    (Wire.read() << 8) |
    Wire.read()
  );

  return true;
}