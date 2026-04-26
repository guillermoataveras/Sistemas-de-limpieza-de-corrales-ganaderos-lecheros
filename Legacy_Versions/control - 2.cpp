#include <Arduino.h>
#include <Wire.h>

// =====================================================
// BUS I2C PARA LOS DOS AS5600
// -----------------------------------------------------
// USANDO SOLO OPCIÓN B — TCA9548A multiplexor I2C
// Un solo bus, el código selecciona canal 0 ó 1
// antes de hablar con cada AS5600.
// =====================================================
static const uint8_t TCA_ADDR   = 0x70;
static const uint8_t TCA_CH_IZQ = 0;   // canal del multiplexor para rueda IZQ
static const uint8_t TCA_CH_DER = 1;   // canal del multiplexor para rueda DER

// =====================================================
// CONFIG ENCODERS AS5600
// =====================================================
static const uint8_t  AS5600_ADDR         = 0x36;
static const uint8_t  AS5600_RAW_ANGLE_H  = 0x0C;  // byte alto del ángulo crudo
static const uint8_t  AS5600_RAW_ANGLE_L  = 0x0D;  // byte bajo del ángulo crudo
static const uint16_t AS5600_MAX_RAW      = 4096;   // 12 bits → 4096 pasos/vuelta

// Intervalo de envío de telemetría de encoders (ms)
static const uint32_t ENC_REPORT_MS       = 20;     // 50 Hz

// =====================================================
// CONFIG GENERAL
// =====================================================
static const uint32_t SERIAL_BAUD = 115200;

// Giro
static const float GYRO_SIGN = 1.0f;              // cambia a -1.0f si sale invertido
static const float GYRO_DEADBAND_DPS = 0.35f;     // zona muerta del gyro
static const float GYRO_ALPHA = 0.25f;            // low-pass
static const uint16_t CALIB_SAMPLES = 1500;

// Envío de DANG solo por cambio
static const float MIN_DELTA_TO_SEND_DEG = 0.20f;
static const uint32_t MAX_SILENCE_MS = 200;

// Seguridad
static const uint32_t CMD_TIMEOUT_MS = 500;

// =====================================================
// CONFIG DE EJE DE YAW
// =====================================================
// AXIS_X = GX, AXIS_Y = GY, AXIS_Z = GZ
static const uint8_t AXIS_X = 0;
static const uint8_t AXIS_Y = 1;
static const uint8_t AXIS_Z = 2;

// Tú ya encontraste que el eje útil era GX
static const uint8_t YAW_GYRO_AXIS = AXIS_X;

// =====================================================
// PINES BTS7960
// =====================================================
// Motor IZQ
static const uint8_t MIZQ_ENAR = 10;
static const uint8_t MIZQ_ENAL = 11;
static const uint8_t MIZQ_PWMR = 5;
static const uint8_t MIZQ_PWML = 4;

// Motor DER
static const uint8_t MDER_ENAR = 8;
static const uint8_t MDER_ENAL = 9;
static const uint8_t MDER_PWMR = 3;
static const uint8_t MDER_PWML = 2;

// Bomba
static const uint8_t BOMBA_ENAL = 36;
static const uint8_t BOMBA_ENAR = 31;
static const uint8_t BOMBA_PWM  = 12;   // el otro PWM está a GND físicamente

// Cepillos
static const uint8_t CEP_ENAL = 29;
static const uint8_t CEP_ENAR = 28;
static const uint8_t CEP_PWM  = 12;     // mismo PWM compartido, según tu cableado

// =====================================================
// MPU9250 - REGISTROS
// =====================================================
static const uint8_t MPU_ADDR = 0x68;

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I     = 0x75;

// =====================================================
// ESTADO DE COMANDOS
// CMD:izq,der,aux_pwm,comp
// =====================================================
struct CommandState {
  int16_t motorIzq = 0;   // -255..255
  int16_t motorDer = 0;   // -255..255
  uint8_t auxPwm = 0;     // 0..255
  bool auxEnable = false; // comp
};

CommandState cmd;
uint32_t lastCmdMillis = 0;

// =====================================================
// ESTADO DEL GIRO
// =====================================================
float gyroBias_dps = 0.0f;
float gyroFiltered_dps = 0.0f;

float pendingDeltaYawDeg = 0.0f;

uint32_t lastGyroMicros = 0;
uint32_t lastTelemetryMillis = 0;

// buffer serial
String rxLine;

// =====================================================
// ESTADO DE ENCODERS AS5600
// =====================================================
// Ángulo crudo anterior de cada rueda (para detectar rollover)
static uint16_t encRawPrev_Izq = 0;
static uint16_t encRawPrev_Der = 0;

// Ángulo acumulado en grados (puede ir más allá de 360, acumulado total)
static float encAccumDeg_Izq  = 0.0f;
static float encAccumDeg_Der  = 0.0f;

// Delta desde el último reporte (se resetea cada ENC_REPORT_MS)
static float encDeltaDeg_Izq  = 0.0f;
static float encDeltaDeg_Der  = 0.0f;

static uint32_t lastEncReportMs = 0;
static bool     encInitialized  = false;

// =====================================================
// ESTRUCTURA RAW IMU
// =====================================================
struct ImuSample {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
};

// =====================================================
// UTILIDADES I2C MPU9250
// =====================================================
bool mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t readCount = Wire.requestFrom(MPU_ADDR, count);
  if (readCount != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
  return true;
}

bool mpuReadWhoAmI(uint8_t &whoami) {
  uint8_t data = 0;
  if (!mpuReadBytes(REG_WHO_AM_I, 1, &data)) {
    return false;
  }
  whoami = data;
  return true;
}

bool mpuReadImuSample(ImuSample &s) {
  uint8_t raw[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, 14, raw)) {
    return false;
  }

  s.accelRaw[0] = (int16_t)((raw[0] << 8) | raw[1]);   // AX
  s.accelRaw[1] = (int16_t)((raw[2] << 8) | raw[3]);   // AY
  s.accelRaw[2] = (int16_t)((raw[4] << 8) | raw[5]);   // AZ

  s.gyroRaw[0]  = (int16_t)((raw[8] << 8) | raw[9]);   // GX
  s.gyroRaw[1]  = (int16_t)((raw[10] << 8) | raw[11]); // GY
  s.gyroRaw[2]  = (int16_t)((raw[12] << 8) | raw[13]); // GZ

  return true;
}

float rawGyroToDps(int16_t raw) {
  return ((float)raw) / 131.0f;   // ±250 dps
}

bool initMPU9250() {
  delay(100);

  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!mpuWriteByte(REG_CONFIG, 0x03)) return false;       // DLPF
  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) return false;  // ±250 dps
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false; // ±2g

  delay(50);
  return true;
}

// =====================================================
// AS5600 — LECTURA DE ÁNGULO CRUDO
// =====================================================

// Selecciona un canal del multiplexor TCA9548A
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Lee el ángulo crudo de 12 bits de un AS5600 usando el bus Wire.
// Con TCA9548A, se llama tcaSelect() antes de cada lectura.
bool as5600ReadRaw(uint16_t &rawOut) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t read_count = Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (read_count != 2) return false;

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  rawOut = ((uint16_t)(hi & 0x0F) << 8) | lo;  // 12 bits válidos
  return true;
}

// Calcula el delta de ángulo en grados manejando rollover.
// AS5600 va de 0 a 4095 y da la vuelta en ambos sentidos.
float as5600DeltaDeg(uint16_t rawPrev, uint16_t rawNow) {
  int16_t diff = (int16_t)rawNow - (int16_t)rawPrev;

  // Corrección de rollover: si el salto es mayor a media vuelta,
  // asumir que cruzó el límite 0/4095
  if (diff >  (int16_t)(AS5600_MAX_RAW / 2)) diff -= AS5600_MAX_RAW;
  if (diff < -(int16_t)(AS5600_MAX_RAW / 2)) diff += AS5600_MAX_RAW;

  // Convertir a grados: 4096 pasos = 360°
  return (float)diff * (360.0f / AS5600_MAX_RAW);
}

// =====================================================
// ACTUALIZACIÓN DE ENCODERS
// Lee ambas ruedas, acumula ángulo, actualiza deltas.
// =====================================================
void updateEncoders() {
  uint16_t rawIzq = 0, rawDer = 0;
  bool okIzq, okDer;

  // Opción B: multiplexor TCA9548A
  tcaSelect(TCA_CH_IZQ);
  okIzq = as5600ReadRaw(rawIzq);

  tcaSelect(TCA_CH_DER);
  okDer = as5600ReadRaw(rawDer);

  // Primera lectura: solo guardar referencia, no acumular
  if (!encInitialized) {
    if (okIzq) encRawPrev_Izq = rawIzq;
    if (okDer)  encRawPrev_Der = rawDer;
    if (okIzq && okDer) encInitialized = true;
    return;
  }

  if (okIzq) {
    float dIzq = as5600DeltaDeg(encRawPrev_Izq, rawIzq);
    encAccumDeg_Izq += dIzq;
    encDeltaDeg_Izq += dIzq;
    encRawPrev_Izq   = rawIzq;
  }

  if (okDer) {
    float dDer = as5600DeltaDeg(encRawPrev_Der, rawDer);
    encAccumDeg_Der += dDer;
    encDeltaDeg_Der += dDer;
    encRawPrev_Der   = rawDer;
  }
}

// =====================================================
// TELEMETRÍA DE ENCODERS
// Envía: ENC:acum_izq,acum_der,delta_izq,delta_der
// Formato: grados con 4 decimales
// =====================================================
void reportEncoders() {
  uint32_t nowMs = millis();
  if ((nowMs - lastEncReportMs) < ENC_REPORT_MS) return;
  lastEncReportMs = nowMs;

  if (!encInitialized) return;

  Serial.print("ENC:");
  Serial.print(encAccumDeg_Izq, 4);
  Serial.print(",");
  Serial.print(encAccumDeg_Der, 4);
  Serial.print(",");
  Serial.print(encDeltaDeg_Izq, 4);
  Serial.print(",");
  Serial.println(encDeltaDeg_Der, 4);

  // Resetear deltas tras el reporte
  encDeltaDeg_Izq = 0.0f;
  encDeltaDeg_Der = 0.0f;
}

// =====================================================
// CALIBRACIÓN GIROSCOPIO
// =====================================================
void calibrateGyro() {
  Serial.println("INFO:Calibrando gyro, deja el robot quieto...");

  float sum = 0.0f;
  uint16_t ok = 0;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    ImuSample s;
    if (mpuReadImuSample(s)) {
      float gyroDps[3] = {
        rawGyroToDps(s.gyroRaw[0]),
        rawGyroToDps(s.gyroRaw[1]),
        rawGyroToDps(s.gyroRaw[2])
      };

      sum += gyroDps[YAW_GYRO_AXIS];
      ok++;
    }
    delay(2);
  }

  if (ok > 0) {
    gyroBias_dps = sum / (float)ok;
  } else {
    gyroBias_dps = 0.0f;
  }

  gyroFiltered_dps = 0.0f;

  Serial.print("INFO:Gyro bias yaw = ");
  Serial.println(gyroBias_dps, 6);
}

// =====================================================
// CONTROL BTS7960
// =====================================================
void driveBTS7960(int16_t value, uint8_t r_en, uint8_t l_en, uint8_t rpwm, uint8_t lpwm) {
  value = constrain(value, -255, 255);

  // En BTS7960 normalmente ambos enables van activos
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);

  if (value == 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
    return;
  }

  uint8_t pwm = (uint8_t)abs(value);

  if (value > 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, pwm);
  }
}

void stopBTS7960(uint8_t r_en, uint8_t l_en, uint8_t rpwm, uint8_t lpwm) {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);
}

// =====================================================
// AUXILIARES
// Asumimos:
// - el otro PWM está a GND físicamente
// - por software solo excitamos un solo PWM
// =====================================================
void driveAuxiliaries(uint8_t pwmValue, bool enableAll) {
  // Habilitamos ambos BTS7960 de auxiliares
  digitalWrite(BOMBA_ENAL, HIGH);
  digitalWrite(BOMBA_ENAR, HIGH);
  digitalWrite(CEP_ENAL, HIGH);
  digitalWrite(CEP_ENAR, HIGH);

  if (!enableAll || pwmValue == 0) {
    analogWrite(BOMBA_PWM, 0);
    return;
  }

  // Como tú tienes el otro PWM a GND físicamente,
  // aquí solo aplicamos el PWM "activo" compartido.
  analogWrite(BOMBA_PWM, pwmValue);
}

// =====================================================
// APLICAR SALIDAS
// =====================================================
void applyOutputs() {
  driveBTS7960(cmd.motorIzq, MIZQ_ENAR, MIZQ_ENAL, MIZQ_PWMR, MIZQ_PWML);
  driveBTS7960(cmd.motorDer, MDER_ENAR, MDER_ENAL, MDER_PWMR, MDER_PWML);
  driveAuxiliaries(cmd.auxPwm, cmd.auxEnable);
}

void stopAllOutputs() {
  cmd.motorIzq = 0;
  cmd.motorDer = 0;
  cmd.auxPwm = 0;
  cmd.auxEnable = false;
  applyOutputs();
}

// =====================================================
// PARSER SERIAL
// Espera:
//   CMD:izq,der,aux_pwm,comp
// Ej:
//   CMD:80,80,255,1
// =====================================================
bool parseCommandLine(const String &line, CommandState &out) {
  if (!line.startsWith("CMD:")) {
    return false;
  }

  String payload = line.substring(4);

  int p1 = payload.indexOf(',');
  if (p1 < 0) return false;
  int p2 = payload.indexOf(',', p1 + 1);
  if (p2 < 0) return false;
  int p3 = payload.indexOf(',', p2 + 1);
  if (p3 < 0) return false;

  String s1 = payload.substring(0, p1);
  String s2 = payload.substring(p1 + 1, p2);
  String s3 = payload.substring(p2 + 1, p3);
  String s4 = payload.substring(p3 + 1);

  out.motorIzq = constrain(s1.toInt(), -255, 255);
  out.motorDer = constrain(s2.toInt(), -255, 255);
  out.auxPwm = (uint8_t)constrain(s3.toInt(), 0, 255);
  out.auxEnable = (s4.toInt() != 0);

  return true;
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      rxLine.trim();

      if (rxLine.length() > 0) {
        CommandState newCmd;
        if (parseCommandLine(rxLine, newCmd)) {
          cmd = newCmd;
          applyOutputs();
          lastCmdMillis = millis();
        }
      }

      rxLine = "";
    } else {
      if (rxLine.length() < 80) {
        rxLine += c;
      } else {
        rxLine = "";
      }
    }
  }
}

// =====================================================
// SEGURIDAD
// =====================================================
void safetyStopIfNoCommand() {
  if ((millis() - lastCmdMillis) > CMD_TIMEOUT_MS) {
    if (cmd.motorIzq != 0 || cmd.motorDer != 0 || cmd.auxPwm != 0 || cmd.auxEnable) {
      stopAllOutputs();
    }
  }
}

// =====================================================
// TELEMETRÍA DE GIRO
// Envía:
//   DANG:<delta_deg>
// Solo cuando hay cambio relevante
// =====================================================
void updateGyroAndSendDelta() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  if (lastGyroMicros == 0) {
    lastGyroMicros = nowUs;
    lastTelemetryMillis = nowMs;
    return;
  }

  ImuSample s;
  if (!mpuReadImuSample(s)) {
    return;
  }

  float dt = (nowUs - lastGyroMicros) * 1e-6f;
  lastGyroMicros = nowUs;

  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  float gyroDps[3] = {
    rawGyroToDps(s.gyroRaw[0]),
    rawGyroToDps(s.gyroRaw[1]),
    rawGyroToDps(s.gyroRaw[2])
  };

  float yawRate_dps = (gyroDps[YAW_GYRO_AXIS] - gyroBias_dps) * GYRO_SIGN;

  gyroFiltered_dps = (GYRO_ALPHA * yawRate_dps) + ((1.0f - GYRO_ALPHA) * gyroFiltered_dps);

  if (fabs(gyroFiltered_dps) < GYRO_DEADBAND_DPS) {
    gyroFiltered_dps = 0.0f;
  }

  float deltaYawDeg = gyroFiltered_dps * dt;
  pendingDeltaYawDeg += deltaYawDeg;

  bool enoughChange = fabs(pendingDeltaYawDeg) >= MIN_DELTA_TO_SEND_DEG;
  bool tooMuchSilence = ((nowMs - lastTelemetryMillis) >= MAX_SILENCE_MS) &&
                        (fabs(pendingDeltaYawDeg) > 0.001f);

  if (enoughChange || tooMuchSilence) {
    Serial.print("DANG:");
    Serial.println(pendingDeltaYawDeg, 6);

    pendingDeltaYawDeg = 0.0f;
    lastTelemetryMillis = nowMs;
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(MIZQ_ENAR, OUTPUT);
  pinMode(MIZQ_ENAL, OUTPUT);
  pinMode(MIZQ_PWMR, OUTPUT);
  pinMode(MIZQ_PWML, OUTPUT);

  pinMode(MDER_ENAR, OUTPUT);
  pinMode(MDER_ENAL, OUTPUT);
  pinMode(MDER_PWMR, OUTPUT);
  pinMode(MDER_PWML, OUTPUT);

  pinMode(BOMBA_ENAL, OUTPUT);
  pinMode(BOMBA_ENAR, OUTPUT);
  pinMode(BOMBA_PWM, OUTPUT);

  pinMode(CEP_ENAL, OUTPUT);
  pinMode(CEP_ENAR, OUTPUT);
  pinMode(CEP_PWM, OUTPUT);

  // Estado inicial BTS7960
  digitalWrite(MIZQ_ENAR, HIGH);
  digitalWrite(MIZQ_ENAL, HIGH);
  digitalWrite(MDER_ENAR, HIGH);
  digitalWrite(MDER_ENAL, HIGH);

  digitalWrite(BOMBA_ENAL, HIGH);
  digitalWrite(BOMBA_ENAR, HIGH);
  digitalWrite(CEP_ENAL, HIGH);
  digitalWrite(CEP_ENAR, HIGH);

  stopAllOutputs();

  Wire.begin();
  Wire.setClock(400000);

  delay(300);

  uint8_t whoami = 0;
  bool whoOk = mpuReadWhoAmI(whoami);

  if (!initMPU9250()) {
    Serial.println("INFO:Error iniciando MPU9250");
  } else {
    Serial.print("INFO:MPU9250 OK. WHOAMI=");
    if (whoOk) {
      Serial.println(whoami, HEX);
    } else {
      Serial.println("??");
    }

    calibrateGyro();
  }

  lastGyroMicros = micros();
  lastTelemetryMillis = millis();
  lastCmdMillis = millis();

  Serial.println("INFO:Sistema listo");
}

void loop() {
  handleSerial();
  safetyStopIfNoCommand();
  updateEncoders();
  reportEncoders();
  updateGyroAndSendDelta();
}