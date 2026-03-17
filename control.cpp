#include <Arduino.h>
#include <Wire.h>

// =====================================================
// CONFIG GENERAL
// =====================================================
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t TELEMETRY_HZ = 50;     // envío de DANG a Python
static const float DT_TARGET = 1.0f / TELEMETRY_HZ;

// Si el yaw sale invertido, cambia a -1.0f
static const float GYRO_SIGN = 1.0f;

// Filtro / compensación gyro
static const float GYRO_DEADBAND_DPS = 0.35f;   // zona muerta en °/s
static const float GYRO_ALPHA = 0.25f;          // low-pass simple
static const uint16_t CALIB_SAMPLES = 1500;

// =====================================================
// PINES
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
static const uint8_t BOMBA_PWM  = 12;   // compartido a propósito

// Cepillos
static const uint8_t CEP_ENAL = 29;
static const uint8_t CEP_ENAR = 28;
static const uint8_t CEP_PWM  = 12;     // compartido a propósito

// =====================================================
// MPU9250 - REGISTROS BÁSICOS
// =====================================================
static const uint8_t MPU_ADDR = 0x68;   // cambia a 0x69 si AD0 está en HIGH

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I     = 0x75;

// =====================================================
// ESTADO DE COMANDOS
// =====================================================
struct CommandState {
  int16_t motorIzq = 0;   // -255..255
  int16_t motorDer = 0;   // -255..255
  uint8_t auxPwm = 0;     // 0..255
  bool auxEnable = false; // comp
};

CommandState cmd;

// =====================================================
// ESTADO DEL GIRO
// =====================================================
float gyroBiasZ_dps = 0.0f;
float gyroFilteredZ_dps = 0.0f;
uint32_t lastGyroMicros = 0;
uint32_t lastTelemetryMicros = 0;

// buffer serial
String rxLine;

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

bool mpuReadGyroZ_dps(float &gz_dps) {
  uint8_t raw[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, 14, raw)) {
    return false;
  }

  int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

  // ±250 dps => 131 LSB/°/s
  gz_dps = ((float)gz_raw) / 131.0f;
  return true;
}

bool initMPU9250() {
  delay(100);

  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) return false; // wake up
  delay(100);

  // DLPF moderado
  if (!mpuWriteByte(REG_CONFIG, 0x03)) return false;

  // GYRO_CONFIG = 0x00 -> ±250 dps
  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) return false;

  // ACCEL_CONFIG = 0x00 -> ±2g (aunque aquí casi no lo usamos)
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;

  delay(50);
  return true;
}

void calibrateGyro() {
  Serial.println("INFO:Calibrando gyro, deja el robot quieto...");

  float sum = 0.0f;
  uint16_t ok = 0;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    float gz;
    if (mpuReadGyroZ_dps(gz)) {
      sum += gz;
      ok++;
    }
    delay(2);
  }

  if (ok > 0) {
    gyroBiasZ_dps = sum / (float)ok;
  } else {
    gyroBiasZ_dps = 0.0f;
  }

  gyroFilteredZ_dps = 0.0f;

  Serial.print("INFO:Gyro bias Z = ");
  Serial.println(gyroBiasZ_dps, 6);
}

// =====================================================
// CONTROL DE MOTORES
// =====================================================
void stopMotorPins(
  uint8_t enA, uint8_t enB,
  uint8_t pwmA, uint8_t pwmB
) {
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}

void driveHBridgeSigned(
  int16_t value,
  uint8_t enR, uint8_t enL,
  uint8_t pwmR, uint8_t pwmL
) {
  value = constrain(value, -255, 255);

  if (value == 0) {
    digitalWrite(enR, LOW);
    digitalWrite(enL, LOW);
    analogWrite(pwmR, 0);
    analogWrite(pwmL, 0);
    return;
  }

  uint8_t pwm = (uint8_t)abs(value);

  if (value > 0) {
    // sentido "adelante"
    digitalWrite(enR, HIGH);
    digitalWrite(enL, LOW);
    analogWrite(pwmR, pwm);
    analogWrite(pwmL, 0);
  } else {
    // sentido "reversa"
    digitalWrite(enR, LOW);
    digitalWrite(enL, HIGH);
    analogWrite(pwmR, 0);
    analogWrite(pwmL, pwm);
  }
}

// Auxiliares compartiendo PWM 12 a propósito
void driveAuxiliaries(uint8_t pwmValue, bool enableAll) {
  if (!enableAll || pwmValue == 0) {
    digitalWrite(BOMBA_ENAL, LOW);
    digitalWrite(BOMBA_ENAR, LOW);
    digitalWrite(CEP_ENAL, LOW);
    digitalWrite(CEP_ENAR, LOW);
    analogWrite(BOMBA_PWM, 0); // mismo pin 12
    return;
  }

  // Bomba y cepillos encendidos al mismo tiempo
  digitalWrite(BOMBA_ENAL, HIGH);
  digitalWrite(BOMBA_ENAR, LOW);

  digitalWrite(CEP_ENAL, HIGH);
  digitalWrite(CEP_ENAR, LOW);

  analogWrite(BOMBA_PWM, pwmValue); // pin 12 compartido
}

void applyOutputs() {
  driveHBridgeSigned(cmd.motorIzq, MIZQ_ENAR, MIZQ_ENAL, MIZQ_PWMR, MIZQ_PWML);
  driveHBridgeSigned(cmd.motorDer, MDER_ENAR, MDER_ENAL, MDER_PWMR, MDER_PWML);
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

  int izq = s1.toInt();
  int der = s2.toInt();
  int aux = s3.toInt();
  int ena = s4.toInt();

  out.motorIzq = constrain(izq, -255, 255);
  out.motorDer = constrain(der, -255, 255);
  out.auxPwm = (uint8_t)constrain(aux, 0, 255);
  out.auxEnable = (ena != 0);

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
// TELEMETRÍA GIRO INCREMENTAL
// Envía:
//   DANG:<delta_deg>
// donde delta_deg corresponde al intervalo desde el último envío
// =====================================================
void updateGyroAndSendDelta() {
  uint32_t nowUs = micros();

  if (lastGyroMicros == 0) {
    lastGyroMicros = nowUs;
    lastTelemetryMicros = nowUs;
    return;
  }

  float gz_dps_raw;
  if (!mpuReadGyroZ_dps(gz_dps_raw)) {
    return;
  }

  float dt = (nowUs - lastGyroMicros) * 1e-6f;
  lastGyroMicros = nowUs;

  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  // quitar bias
  float gz_dps = (gz_dps_raw - gyroBiasZ_dps) * GYRO_SIGN;

  // low pass
  gyroFilteredZ_dps = (GYRO_ALPHA * gz_dps) + ((1.0f - GYRO_ALPHA) * gyroFilteredZ_dps);

  // deadband
  if (fabs(gyroFilteredZ_dps) < GYRO_DEADBAND_DPS) {
    gyroFilteredZ_dps = 0.0f;
  }

  // mandar a TELEMETRY_HZ
  uint32_t telemetryPeriodUs = 1000000UL / TELEMETRY_HZ;
  if ((nowUs - lastTelemetryMicros) >= telemetryPeriodUs) {
    float dtTelemetry = (nowUs - lastTelemetryMicros) * 1e-6f;
    lastTelemetryMicros = nowUs;

    float deltaYawDeg = gyroFilteredZ_dps * dtTelemetry;

    Serial.print("DANG:");
    Serial.println(deltaYawDeg, 6);
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
  lastTelemetryMicros = lastGyroMicros;

  Serial.println("INFO:Sistema listo");
}

void loop() {
  handleSerial();
  updateGyroAndSendDelta();
}