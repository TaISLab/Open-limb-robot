/*
  Lectura del sensor conectado en A0
  Arduino Mega 2560

  Formato enviado:
  DATA,tiempo_ms,ADC,voltaje,bloque

  El Arduino espera el comando START enviado desde MATLAB.
*/

const uint8_t SENSOR_PIN = A0;

const unsigned long PERIODO_MUESTREO_MS = 100;  // 10 muestras por segundo
const unsigned long INTERVALO_PESO_MS   = 5000; // Nuevo bloque cada 5 segundos

// Voltaje de referencia nominal del ADC.
// Puede ajustarse después si se mide un valor diferente.
const float VOLTAJE_REFERENCIA = 5.0;

bool adquiriendo = false;

unsigned long tiempoInicio = 0;
unsigned long proximaMuestra = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(SENSOR_PIN, INPUT);

  // Mensaje de arranque. MATLAB lo eliminará antes de comenzar.
  Serial.println("ARDUINO_READY");
}

void loop()
{
  revisarComandos();

  if (!adquiriendo)
  {
    return;
  }

  unsigned long tiempoTranscurrido = millis() - tiempoInicio;

  if (tiempoTranscurrido >= proximaMuestra)
  {
    tomarYEnviarMuestra(tiempoTranscurrido);

    proximaMuestra += PERIODO_MUESTREO_MS;

    // Evita acumulaciones si por alguna razón hubo un retraso grande.
    if (tiempoTranscurrido > proximaMuestra + PERIODO_MUESTREO_MS)
    {
      proximaMuestra = tiempoTranscurrido + PERIODO_MUESTREO_MS;
    }
  }
}

void revisarComandos()
{
  if (Serial.available() <= 0)
  {
    return;
  }

  String comando = Serial.readStringUntil('\n');
  comando.trim();

  if (comando == "START")
  {
    tiempoInicio = millis();
    proximaMuestra = 0;
    adquiriendo = true;

    Serial.println("READY");
  }
  else if (comando == "STOP")
  {
    adquiriendo = false;
    Serial.println("STOPPED");
  }
}

void tomarYEnviarMuestra(unsigned long tiempoMs)
{
  int lecturaADC = analogRead(SENSOR_PIN);

  float voltaje =
    lecturaADC * VOLTAJE_REFERENCIA / 1023.0;

  unsigned long bloque =
    tiempoMs / INTERVALO_PESO_MS;

  Serial.print("DATA,");
  Serial.print(tiempoMs);
  Serial.print(",");
  Serial.print(lecturaADC);
  Serial.print(",");
  Serial.print(voltaje, 6);
  Serial.print(",");
  Serial.println(bloque);
}