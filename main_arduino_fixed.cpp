#include <Arduino.h>
#include <Wire.h>

// =====================================================
// BUS I2C PARA LOS DOS AS5600
// TCA9548A (HW617) multiplexor I2C — dirección 0x70
// Canal 6 = rueda IZQ, Canal 7 = rueda DER
// =====================================================
static const uint8_t TCA_ADDR   = 0x70;
static const uint8_t TCA_CH_IZQ = 6;
static const uint8_t TCA_CH_DER = 7;

// =====================================================
// PINES DIR AS5600
// =====================================================
static const uint8_t ENC_DIR_IZQ = 7;
static const uint8_t ENC_DIR_DER = 6;
static const uint8_t ENC_DIR_IZQ_LEVEL = LOW;
static const uint8_t ENC_DIR_DER_LEVEL = HIGH;

// =====================================================
// CONFIG ENCODERS AS5600
// =====================================================
static const uint8_t  AS5600_ADDR        = 0x36;
static const uint8_t  AS5600_RAW_ANGLE_H = 0x0C;
static const uint8_t  AS5600_RAW_ANGLE_L = 0x0D;
static const uint16_t AS5600_MAX_RAW     = 4096;

static const uint32_t ENC_REPORT_MS = 20;
static const uint32_t ENC_READ_MS   = 20;

static const uint8_t  ENC_MAX_FAILS = 3;
static const uint32_t ENC_RETRY_MS  = 1000;

// =====================================================
// CONFIG GENERAL
// =====================================================
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t I2C_TIMEOUT_US = 3000;

// ═══════════════════════════════════════════════════════
// GIRO — SENTIDO DE REPORTE DEL YAW
// ═══════════════════════════════════════════════════════
// >>> CAMBIA AQUÍ EL SIGNO PARA INVERTIR EL SENTIDO DEL YAW <<<
// GYRO_SIGN = +1.0f  →  sentido reportado actual
// GYRO_SIGN = -1.0f  →  invierte el sentido (usa si Python ve giros al revés)
//
// Alternativamente, si el MPU está montado con otro eje apuntando hacia
// arriba, cambia YAW_GYRO_AXIS más abajo.
// ═══════════════════════════════════════════════════════
static const float GYRO_SIGN = 1.0f;

static const float    GYRO_DEADBAND_DPS   = 0.35f;
static const float    GYRO_ALPHA          = 0.25f;

// Reducido de 1500 a 500 — evita bloquear 3s al arrancar
static const uint16_t CALIB_SAMPLES       = 500;

static const float    MIN_DELTA_TO_SEND_DEG = 0.20f;
static const uint32_t MAX_SILENCE_MS        = 200;

static const uint32_t CMD_TIMEOUT_MS = 500;
static const uint32_t GYRO_READ_MS   = 5;

// Opcional: enviar ACK cuando se recibe un CMD válido (para depurar)
// Formato: ACK:izq,der,aux,comp,bomba_en,cep_en
static const bool ACK_COMMANDS       = true;

// =====================================================
// CONFIG DE EJE DE YAW
// =====================================================
static const uint8_t AXIS_X = 0;
static const uint8_t AXIS_Y = 1;
static const uint8_t AXIS_Z = 2;
// >>> Cambia aquí si el MPU9250 no está con el eje Z hacia arriba <<<
static const uint8_t YAW_GYRO_AXIS = AXIS_Z;

// =====================================================
// PINES BTS7960
// =====================================================
static const uint8_t MIZQ_ENAR = 5;
static const uint8_t MIZQ_ENAL = 4;
static const uint8_t MIZQ_PWMR = 3;
static const uint8_t MIZQ_PWML = 2;

static const uint8_t MDER_ENAR = 11;
static const uint8_t MDER_ENAL = 10;
static const uint8_t MDER_PWMR = 9;
static const uint8_t MDER_PWML = 8;

// Bomba
static const uint8_t BOMBA_ENAL = 39;
static const uint8_t BOMBA_ENAR = 47;
static const uint8_t BOMBA_PWM  = 12;

// Cepillos (comparten PWM con bomba según cableado)
static const uint8_t CEP_ENAL = 35;
static const uint8_t CEP_ENAR = 45;
static const uint8_t CEP_PWM  = 12;

// =====================================================
// MPU9250
// =====================================================
static const uint8_t MPU_ADDR         = 0x68;
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
  int16_t motorIzq  = 0;
  int16_t motorDer  = 0;
  uint8_t auxPwm    = 0;
  bool    auxEnable = false;
};

CommandState cmd;
uint32_t lastCmdMillis = 0;

// =====================================================
// ESTADO DEL GIRO
// =====================================================
float gyroBias_dps       = 0.0f;
float gyroFiltered_dps   = 0.0f;
float pendingDeltaYawDeg = 0.0f;

uint32_t lastGyroMicros      = 0;
uint32_t lastTelemetryMillis = 0;
uint32_t lastGyroReadMs      = 0;

String rxLine;

// =====================================================
// ESTADO DE ENCODERS AS5600
// =====================================================
static uint16_t encRawPrev_Izq = 0;
static uint16_t encRawPrev_Der = 0;

static float encAccumDeg_Izq = 0.0f;
static float encAccumDeg_Der = 0.0f;

static float encDeltaDeg_Izq = 0.0f;
static float encDeltaDeg_Der = 0.0f;

static uint32_t lastEncReportMs = 0;
static uint32_t lastEncReadMs   = 0;

static bool     encInitialized  = false;
static bool     encodersEnabled = true;

static uint8_t  encFailCountIzq = 0;
static uint8_t  encFailCountDer = 0;
static uint32_t encRetryAtMs    = 0;

// =====================================================
// ESTADO IMU
// =====================================================
static bool imuEnabled = true;
static uint8_t imuFailCount = 0;
static const uint8_t  IMU_MAX_FAILS = 5;
static const uint32_t IMU_RETRY_MS  = 1000;
static uint32_t imuRetryAtMs = 0;

struct ImuSample {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
};

// =====================================================
// UTILIDADES I2C
// =====================================================
void flushWireRxBuffer() {
  while (Wire.available()) (void)Wire.read();
}

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
    flushWireRxBuffer();
    return false;
  }
  uint8_t readCount = Wire.requestFrom((uint8_t)MPU_ADDR, count, (uint8_t)true);
  if (readCount != count) {
    flushWireRxBuffer();
    return false;
  }
  for (uint8_t i = 0; i < count; i++) {
    if (!Wire.available()) { flushWireRxBuffer(); return false; }
    dest[i] = Wire.read();
  }
  return true;
}

bool mpuReadWhoAmI(uint8_t &whoami) {
  uint8_t data = 0;
  if (!mpuReadBytes(REG_WHO_AM_I, 1, &data)) return false;
  whoami = data;
  return true;
}

bool mpuReadImuSample(ImuSample &s) {
  uint8_t raw[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, 14, raw)) return false;
  s.accelRaw[0] = (int16_t)((raw[0]  << 8) | raw[1]);
  s.accelRaw[1] = (int16_t)((raw[2]  << 8) | raw[3]);
  s.accelRaw[2] = (int16_t)((raw[4]  << 8) | raw[5]);
  s.gyroRaw[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
  s.gyroRaw[1]  = (int16_t)((raw[10] << 8) | raw[11]);
  s.gyroRaw[2]  = (int16_t)((raw[12] << 8) | raw[13]);
  return true;
}

float rawGyroToDps(int16_t raw) { return ((float)raw) / 131.0f; }

bool initMPU9250() {
  delay(100);
  if (!mpuWriteByte(REG_PWR_MGMT_1,   0x00)) return false;
  delay(100);
  if (!mpuWriteByte(REG_CONFIG,       0x03)) return false;
  if (!mpuWriteByte(REG_GYRO_CONFIG,  0x00)) return false;
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;
  delay(50);
  return true;
}

// =====================================================
// AS5600
// =====================================================
bool tcaSelectSafe(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write((uint8_t)(1 << channel));
  return (Wire.endTransmission() == 0);
}

bool tcaDisableAllSafe() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write((uint8_t)0x00);
  return (Wire.endTransmission() == 0);
}

bool as5600ReadRawSafe(uint16_t &rawOut) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) { flushWireRxBuffer(); return false; }
  uint8_t n = Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2, (uint8_t)true);
  if (n != 2) { flushWireRxBuffer(); return false; }
  if (Wire.available() < 2) { flushWireRxBuffer(); return false; }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  rawOut = ((uint16_t)(hi & 0x0F) << 8) | lo;
  return true;
}

float as5600DeltaDeg(uint16_t rawPrev, uint16_t rawNow) {
  int16_t diff = (int16_t)rawNow - (int16_t)rawPrev;
  if (diff >  (int16_t)(AS5600_MAX_RAW / 2)) diff -= AS5600_MAX_RAW;
  if (diff < -(int16_t)(AS5600_MAX_RAW / 2)) diff += AS5600_MAX_RAW;
  return (float)diff * (360.0f / AS5600_MAX_RAW);
}

void diagEncoders() {
  Serial.println("INFO:--- Diagnostico AS5600 ---");
  uint16_t rawTest = 0;
  if (tcaSelectSafe(TCA_CH_IZQ) && as5600ReadRawSafe(rawTest)) {
    Serial.print("INFO:AS5600 IZQ OK  raw="); Serial.println(rawTest);
  } else {
    Serial.println("INFO:AS5600 IZQ FALLO — revisar cableado canal 6");
  }
  if (tcaSelectSafe(TCA_CH_DER) && as5600ReadRawSafe(rawTest)) {
    Serial.print("INFO:AS5600 DER OK  raw="); Serial.println(rawTest);
  } else {
    Serial.println("INFO:AS5600 DER FALLO — revisar cableado canal 7");
  }
  tcaDisableAllSafe();
  Serial.println("INFO:--- Fin diagnostico ---");
}

void updateEncoders() {
  uint32_t nowMs = millis();
  if (!encodersEnabled) {
    if (nowMs < encRetryAtMs) return;
    encodersEnabled = true;
    encFailCountIzq = 0;
    encFailCountDer = 0;
    Serial.println("INFO:Reintentando encoders...");
  }

  if ((nowMs - lastEncReadMs) < ENC_READ_MS) return;
  lastEncReadMs = nowMs;

  uint16_t rawIzq = 0, rawDer = 0;
  bool okIzq = false, okDer = false;

  if (tcaSelectSafe(TCA_CH_IZQ)) okIzq = as5600ReadRawSafe(rawIzq);
  if (tcaSelectSafe(TCA_CH_DER)) okDer = as5600ReadRawSafe(rawDer);
  tcaDisableAllSafe();

  if (!encInitialized) {
    if (okIzq) encRawPrev_Izq = rawIzq;
    if (okDer) encRawPrev_Der = rawDer;
    if (okIzq && okDer) {
      encInitialized = true;
      encFailCountIzq = 0;
      encFailCountDer = 0;
      Serial.println("INFO:Encoders inicializados OK");
    } else {
      if (!okIzq) { if (encFailCountIzq < 255) encFailCountIzq++; }
      if (!okDer) { if (encFailCountDer < 255) encFailCountDer++; }
    }
    return;
  }

  if (okIzq) {
    float dIzq = as5600DeltaDeg(encRawPrev_Izq, rawIzq);
    encAccumDeg_Izq += dIzq;
    encDeltaDeg_Izq += dIzq;
    encRawPrev_Izq = rawIzq;
    encFailCountIzq = 0;
  } else {
    if (encFailCountIzq < 255) encFailCountIzq++;
  }

  if (okDer) {
    float dDer = as5600DeltaDeg(encRawPrev_Der, rawDer);
    encAccumDeg_Der += dDer;
    encDeltaDeg_Der += dDer;
    encRawPrev_Der = rawDer;
    encFailCountDer = 0;
  } else {
    if (encFailCountDer < 255) encFailCountDer++;
  }

  if (encFailCountIzq >= ENC_MAX_FAILS || encFailCountDer >= ENC_MAX_FAILS) {
    encodersEnabled = false;
    encRetryAtMs = nowMs + ENC_RETRY_MS;
    Serial.println("WARN:Encoders deshabilitados temporalmente por fallo I2C");
  }
}

void reportEncoders() {
  uint32_t nowMs = millis();
  if ((nowMs - lastEncReportMs) < ENC_REPORT_MS) return;
  lastEncReportMs = nowMs;
  if (!encInitialized) return;
  Serial.print("ENC:");
  Serial.print(encAccumDeg_Izq, 4); Serial.print(",");
  Serial.print(encAccumDeg_Der, 4); Serial.print(",");
  Serial.print(encDeltaDeg_Izq, 4); Serial.print(",");
  Serial.println(encDeltaDeg_Der, 4);
  encDeltaDeg_Izq = 0.0f;
  encDeltaDeg_Der = 0.0f;
}

void calibrateGyro() {
  Serial.println("INFO:Calibrando gyro, deja el robot quieto...");
  float sum = 0.0f; uint16_t ok = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    ImuSample s;
    if (mpuReadImuSample(s)) {
      sum += rawGyroToDps(s.gyroRaw[YAW_GYRO_AXIS]);
      ok++;
    }
    delay(2);
  }
  gyroBias_dps = (ok > 0) ? (sum / (float)ok) : 0.0f;
  gyroFiltered_dps = 0.0f;
  Serial.print("INFO:Gyro bias yaw = "); Serial.println(gyroBias_dps, 6);
}

// =====================================================
// CONTROL BTS7960
// =====================================================
void driveBTS7960(int16_t value,
                  uint8_t r_en, uint8_t l_en,
                  uint8_t rpwm, uint8_t lpwm) {
  value = constrain(value, -255, 255);
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  if (value == 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
    return;
  }
  uint8_t pwm = (uint8_t)abs(value);
  if (value > 0) { analogWrite(rpwm, pwm); analogWrite(lpwm, 0); }
  else            { analogWrite(rpwm, 0); analogWrite(lpwm, pwm); }
}

void stopBTS7960(uint8_t r_en, uint8_t l_en, uint8_t rpwm, uint8_t lpwm) {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);
}

// =====================================================
// AUXILIARES (bomba + cepillos) — CORREGIDO
// Antes: solo BOMBA_PWM se escribía, ni siquiera se tocaba CEP_PWM
// (aunque ambos apunten al pin 12).
// Ahora: se escribe a AMBOS pines explícitamente + diagnóstico.
// =====================================================
void driveAuxiliaries(uint8_t pwmValue, bool enableAll) {
  // Enables siempre HIGH — el BTS7960 los requiere para operar
  digitalWrite(BOMBA_ENAL, HIGH);
  digitalWrite(BOMBA_ENAR, HIGH);
  digitalWrite(CEP_ENAL,   HIGH);
  digitalWrite(CEP_ENAR,   HIGH);

  if (!enableAll || pwmValue == 0) {
    analogWrite(BOMBA_PWM, 0);
    analogWrite(CEP_PWM,   0);
    return;
  }

  // Escribir explícitamente a ambos pines aunque compartan número
  analogWrite(BOMBA_PWM, pwmValue);
  analogWrite(CEP_PWM,   pwmValue);
}

void applyOutputs() {
  driveBTS7960(cmd.motorIzq, MIZQ_ENAR, MIZQ_ENAL, MIZQ_PWMR, MIZQ_PWML);
  driveBTS7960(cmd.motorDer, MDER_ENAR, MDER_ENAL, MDER_PWMR, MDER_PWML);
  driveAuxiliaries(cmd.auxPwm, cmd.auxEnable);
}

void stopAllOutputs() {
  cmd.motorIzq  = 0;
  cmd.motorDer  = 0;
  cmd.auxPwm    = 0;
  cmd.auxEnable = false;
  applyOutputs();
}

// =====================================================
// PARSER SERIAL — Formato: CMD:izq,der,aux_pwm,comp
// =====================================================
bool parseCommandLine(const String &line, CommandState &out) {
  if (!line.startsWith("CMD:")) return false;
  String payload = line.substring(4);
  int p1 = payload.indexOf(',');       if (p1 < 0) return false;
  int p2 = payload.indexOf(',', p1+1); if (p2 < 0) return false;
  int p3 = payload.indexOf(',', p2+1); if (p3 < 0) return false;
  out.motorIzq  = constrain(payload.substring(0,      p1).toInt(), -255, 255);
  out.motorDer  = constrain(payload.substring(p1 + 1, p2).toInt(), -255, 255);
  out.auxPwm    = (uint8_t)constrain(payload.substring(p2 + 1, p3).toInt(), 0, 255);
  out.auxEnable = (payload.substring(p3 + 1).toInt() != 0);
  return true;
}

// =====================================================
// ACK — diagnóstico opcional
// Formato: ACK:izq,der,aux,comp,bomba_pin_hi,cep_pin_hi
// =====================================================
void sendAck() {
  if (!ACK_COMMANDS) return;
  Serial.print("ACK:");
  Serial.print(cmd.motorIzq);                  Serial.print(",");
  Serial.print(cmd.motorDer);                  Serial.print(",");
  Serial.print(cmd.auxPwm);                    Serial.print(",");
  Serial.print(cmd.auxEnable ? 1 : 0);         Serial.print(",");
  Serial.print(digitalRead(BOMBA_ENAL));       Serial.print(",");
  Serial.println(digitalRead(CEP_ENAL));
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
          sendAck();
        }
      }
      rxLine = "";
    } else {
      if (rxLine.length() < 80) rxLine += c;
      else                      rxLine = "";
    }
  }
}

void safetyStopIfNoCommand() {
  if ((millis() - lastCmdMillis) > CMD_TIMEOUT_MS) {
    if (cmd.motorIzq != 0 || cmd.motorDer != 0 ||
        cmd.auxPwm   != 0 || cmd.auxEnable) {
      stopAllOutputs();
    }
  }
}

// =====================================================
// TELEMETRÍA DE GIRO (DANG:)
// >>> El signo final respeta GYRO_SIGN definido arriba. <<<
// =====================================================
void updateGyroAndSendDelta() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  if (!imuEnabled) {
    if (nowMs < imuRetryAtMs) return;
    uint8_t whoami = 0;
    if (mpuReadWhoAmI(whoami) && initMPU9250()) {
      imuEnabled = true;
      imuFailCount = 0;
      lastGyroMicros = nowUs;
      lastTelemetryMillis = nowMs;
      Serial.println("INFO:IMU reactivada");
    } else {
      imuRetryAtMs = nowMs + IMU_RETRY_MS;
      return;
    }
  }

  if ((nowMs - lastGyroReadMs) < GYRO_READ_MS) return;
  lastGyroReadMs = nowMs;

  if (lastGyroMicros == 0) {
    lastGyroMicros = nowUs;
    lastTelemetryMillis = nowMs;
    return;
  }

  ImuSample s;
  if (!mpuReadImuSample(s)) {
    if (imuFailCount < 255) imuFailCount++;
    if (imuFailCount >= IMU_MAX_FAILS) {
      imuEnabled = false;
      imuRetryAtMs = nowMs + IMU_RETRY_MS;
      Serial.println("WARN:IMU deshabilitada temporalmente por fallo I2C");
    }
    return;
  }
  imuFailCount = 0;

  float dt = (nowUs - lastGyroMicros) * 1e-6f;
  lastGyroMicros = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return;

  // Yaw rate bruto del eje elegido, con bias restado y SIGNO aplicado aquí:
  float yawRateRaw_dps = rawGyroToDps(s.gyroRaw[YAW_GYRO_AXIS]);
  float yawRate_dps    = (yawRateRaw_dps - gyroBias_dps) * GYRO_SIGN;

  gyroFiltered_dps = (GYRO_ALPHA * yawRate_dps) +
                     ((1.0f - GYRO_ALPHA) * gyroFiltered_dps);

  if (fabs(gyroFiltered_dps) < GYRO_DEADBAND_DPS) gyroFiltered_dps = 0.0f;

  float deltaYawDeg = gyroFiltered_dps * dt;
  pendingDeltaYawDeg += deltaYawDeg;

  bool enoughChange   = fabs(pendingDeltaYawDeg) >= MIN_DELTA_TO_SEND_DEG;
  bool tooMuchSilence = ((nowMs - lastTelemetryMillis) >= MAX_SILENCE_MS) &&
                        (fabs(pendingDeltaYawDeg) > 0.001f);

  if (enoughChange || tooMuchSilence) {
    Serial.print("DANG:");
    Serial.println(pendingDeltaYawDeg, 6);
    pendingDeltaYawDeg  = 0.0f;
    lastTelemetryMillis = nowMs;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // DIR pins del AS5600 PRIMERO
  pinMode(ENC_DIR_IZQ, OUTPUT);
  pinMode(ENC_DIR_DER, OUTPUT);
  digitalWrite(ENC_DIR_IZQ, ENC_DIR_IZQ_LEVEL);
  digitalWrite(ENC_DIR_DER, ENC_DIR_DER_LEVEL);
  delay(10);

  // Pines motores
  pinMode(MIZQ_ENAR, OUTPUT); pinMode(MIZQ_ENAL, OUTPUT);
  pinMode(MIZQ_PWMR, OUTPUT); pinMode(MIZQ_PWML, OUTPUT);
  pinMode(MDER_ENAR, OUTPUT); pinMode(MDER_ENAL, OUTPUT);
  pinMode(MDER_PWMR, OUTPUT); pinMode(MDER_PWML, OUTPUT);

  // Pines auxiliares
  pinMode(BOMBA_ENAL, OUTPUT); pinMode(BOMBA_ENAR, OUTPUT);
  pinMode(BOMBA_PWM,  OUTPUT);
  pinMode(CEP_ENAL,   OUTPUT); pinMode(CEP_ENAR,   OUTPUT);
  pinMode(CEP_PWM,    OUTPUT);

  // Enables HIGH de inicio (BTS7960 requiere esto para operar)
  digitalWrite(MIZQ_ENAR, HIGH);  digitalWrite(MIZQ_ENAL, HIGH);
  digitalWrite(MDER_ENAR, HIGH);  digitalWrite(MDER_ENAL, HIGH);
  digitalWrite(BOMBA_ENAL, HIGH); digitalWrite(BOMBA_ENAR, HIGH);
  digitalWrite(CEP_ENAL,   HIGH); digitalWrite(CEP_ENAR,   HIGH);

  stopAllOutputs();

  // I2C
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  delay(300);

  // MPU9250
  uint8_t whoami = 0;
  bool whoOk = mpuReadWhoAmI(whoami);
  if (!initMPU9250()) {
    imuEnabled = false;
    imuRetryAtMs = millis() + IMU_RETRY_MS;
    Serial.println("INFO:Error iniciando MPU9250");
  } else {
    Serial.print("INFO:MPU9250 OK. WHOAMI=");
    Serial.println(whoOk ? String(whoami, HEX) : String("??"));
    calibrateGyro();
  }

  diagEncoders();

  lastGyroMicros      = micros();
  lastTelemetryMillis = millis();
  lastCmdMillis       = millis();
  lastEncReadMs       = 0;
  lastGyroReadMs      = 0;

  Serial.println("INFO:Sistema listo");
  // Reportar estado inicial de enables auxiliares (diagnóstico)
  Serial.print("INFO:ENABLES bomba_EL=");   Serial.print(digitalRead(BOMBA_ENAL));
  Serial.print(" bomba_ER=");               Serial.print(digitalRead(BOMBA_ENAR));
  Serial.print(" cep_EL=");                 Serial.print(digitalRead(CEP_ENAL));
  Serial.print(" cep_ER=");                 Serial.println(digitalRead(CEP_ENAR));
}

void loop() {
  handleSerial();
  safetyStopIfNoCommand();
  updateEncoders();
  reportEncoders();
  updateGyroAndSendDelta();
  handleSerial();
  safetyStopIfNoCommand();
}
