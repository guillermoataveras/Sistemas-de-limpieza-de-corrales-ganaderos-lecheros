// ═══════════════════════════════════════════════════════════════
// FIRMWARE ARDUINO — Robot limpiador de corrales
// ───────────────────────────────────────────────────────────────
// Este firmware corre en un Arduino Mega y se encarga de:
//   1. Recibir comandos del programa Python por Serial (USB)
//      Formato: CMD:motor_izq,motor_der,aux_pwm,bomba_enable,cep_enable,relay_open
//   2. Controlar los motores de tracción con dos drivers BTS7960
//   3. Controlar la bomba y cepillos con un tercer driver BTS7960
//   4. Controlar un relé normalmente cerrado del compresor en D24
//   5. Leer el giroscopio MPU9250 por I2C y reportar el delta de yaw
//      al Python usando el mensaje "DANG:" cada vez que cambia
//      significativamente.
//
// NOTA: El soporte de encoders AS5600 + multiplexor TCA9548A fue
// removido en esta versión. El código antiguo queda comentado para
// referencia en caso de que se quiera volver a activar más adelante.
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>


// ═══════════════════════════════════════════════════════
// CONFIGURACIÓN GENERAL
// ═══════════════════════════════════════════════════════

// Velocidad del puerto serial USB hacia el Python (baudios)
static const uint32_t SERIAL_BAUD = 115200;

// Timeout del periférico I2C en microsegundos. Si el bus se cuelga
// (por ejemplo por un sensor que no responde), Wire aborta la
// operación y libera el loop principal. Evita freezes del Arduino.
static const uint32_t I2C_TIMEOUT_US = 3000;

// Tiempo sin recibir comandos antes de que el robot se detenga por
// seguridad. Si Python se congela o se pierde el cable USB, el
// Arduino apaga motores automáticamente tras este tiempo (ms).
static const uint32_t CMD_TIMEOUT_MS = 500;


// ═══════════════════════════════════════════════════════
// GIRO — CONFIGURACIÓN DEL GIROSCOPIO MPU9250
// ═══════════════════════════════════════════════════════

// ┌─────────────────────────────────────────────────────────────┐
// │  >>> CAMBIA ESTE SIGNO PARA INVERTIR EL SENTIDO DEL YAW <<<  │
// │  GYRO_SIGN = +1.0f  →  sentido reportado actual              │
// │  GYRO_SIGN = -1.0f  →  invierte el sentido reportado         │
// │                                                              │
// │  Usa -1.0f si el Python ve los giros al revés.               │
// │  Alternativamente, cambia el eje del gyro (YAW_GYRO_AXIS)    │
// │  si el MPU no está con el eje Z apuntando hacia arriba.      │
// └─────────────────────────────────────────────────────────────┘
static const float GYRO_SIGN = 1.0f;

// Zona muerta en grados por segundo. Si la velocidad angular es
// menor que esto, se considera ruido y se fija a 0 (evita drift
// acumulado cuando el robot está quieto).
static const float GYRO_DEADBAND_DPS = 0.35f;

// Filtro pasa-bajos (EMA) para suavizar el yaw_rate. Valor entre
// 0 y 1. Más alto = más responsivo pero más ruidoso.
static const float GYRO_ALPHA = 0.25f;

// Número de muestras usadas al inicio para calcular el bias del
// giroscopio. Cada muestra tarda ~2ms, así que 500 muestras = 1s
// de bloqueo en el setup. Mayor valor = mejor calibración.
static const uint16_t CALIB_SAMPLES = 500;

// Cuánto tiene que haber acumulado el yaw para enviar un DANG:
// al Python (en grados). Evita saturar el serial con cambios
// imperceptibles.
static const float MIN_DELTA_TO_SEND_DEG = 0.20f;

// Tiempo máximo sin enviar DANG: antes de forzar un envío aunque
// el cambio sea pequeño. Mantiene el heartbeat de yaw vivo (ms).
static const uint32_t MAX_SILENCE_MS = 200;

// Cada cuántos ms se lee el giroscopio. 5ms = 200 Hz aprox, más
// que suficiente para navegación (el bucle principal suele correr
// a 30-60 Hz).
static const uint32_t GYRO_READ_MS = 5;

// Ejes del MPU9250 — no tocar, son para claridad
static const uint8_t AXIS_X = 0;
static const uint8_t AXIS_Y = 1;
static const uint8_t AXIS_Z = 2;

// ┌─────────────────────────────────────────────────────────────┐
// │  >>> Cambia AXIS_Z si el MPU no está con Z hacia arriba <<< │
// └─────────────────────────────────────────────────────────────┘
static const uint8_t YAW_GYRO_AXIS = AXIS_Z;


// ═══════════════════════════════════════════════════════
// DEPURACIÓN — ACK de comandos recibidos
// ═══════════════════════════════════════════════════════

// Si está en true, cada vez que Arduino reciba un CMD: válido
// responderá con una línea ACK: mostrando lo que aplicó.
// Útil para debug. Pon false para reducir tráfico serial.
// Formato: ACK:izq,der,aux,bomba_enable,cep_enable,relay_open,bomba_EL,cep_EL
static const bool ACK_COMMANDS = true;


// ═══════════════════════════════════════════════════════
// PINES DE MOTORES DE TRACCIÓN (DRIVERS BTS7960)
// ═══════════════════════════════════════════════════════
// Cada motor requiere 4 pines:
//   - R_EN y L_EN: habilitadores (HIGH = driver activo)
//   - RPWM: PWM para girar en un sentido
//   - LPWM: PWM para girar en el sentido opuesto

// Motor IZQUIERDO
static const uint8_t MIZQ_ENAR = 5;   // Enable derecho
static const uint8_t MIZQ_ENAL = 4;   // Enable izquierdo
static const uint8_t MIZQ_PWMR = 3;   // PWM adelante
static const uint8_t MIZQ_PWML = 2;   // PWM atrás

// Motor DERECHO
static const uint8_t MDER_ENAR = 11;
static const uint8_t MDER_ENAL = 10;
static const uint8_t MDER_PWMR = 9;
static const uint8_t MDER_PWML = 8;


// ═══════════════════════════════════════════════════════
// PINES DE BOMBA Y CEPILLOS (TERCER DRIVER BTS7960)
// ═══════════════════════════════════════════════════════
// Bomba de agua y cepillos comparten el pin de PWM (pin 12),
// cada uno tiene sus propios enables.

// Bomba
static const uint8_t BOMBA_ENAL = 39;
static const uint8_t BOMBA_ENAR = 47;
static const uint8_t BOMBA_PWM  = 12;

// Cepillos (comparten pin PWM con bomba — así es el cableado)
static const uint8_t CEP_ENAL = 35;
static const uint8_t CEP_ENAR = 45;
static const uint8_t CEP_PWM  = 12;


// ═══════════════════════════════════════════════════════
// RELÉ DE EMERGENCIA DEL COMPRESOR (PIN D24)
// ═══════════════════════════════════════════════════════
// Relé NORMALMENTE CERRADO. Mientras no se haga nada, el
// compresor está alimentado. Python puede abrir el relé
// (corta el compresor) vía el campo relay_open del comando CMD:.
//
// Lógica del relé (active-LOW):
//   - D24 = LOW   → relé CERRADO → compresor ALIMENTADO (estado normal)
//   - D24 = HIGH  → relé ABIERTO → compresor SIN ALIMENTAR
//
// Cambia RELAY_OPEN_LEVEL si tu módulo relé es active-HIGH.

static const uint8_t RELAY_COMPRESOR_PIN = 24;    // D24
static const uint8_t RELAY_OPEN_LEVEL    = HIGH;  // nivel para ABRIR (corte)
static const uint8_t RELAY_CLOSED_LEVEL  = LOW;   // nivel para CERRAR (pasa)


// ═══════════════════════════════════════════════════════
// REGISTROS DEL MPU9250
// ═══════════════════════════════════════════════════════
// Direcciones I2C de los registros internos del chip.
// No necesitas tocarlos salvo para cambiar rangos del giroscopio.

static const uint8_t MPU_ADDR         = 0x68;   // dirección I2C del MPU
static const uint8_t REG_PWR_MGMT_1   = 0x6B;   // control de power / reset
static const uint8_t REG_CONFIG       = 0x1A;   // filtro digital (DLPF)
static const uint8_t REG_GYRO_CONFIG  = 0x1B;   // rango del giroscopio
static const uint8_t REG_ACCEL_CONFIG = 0x1C;   // rango del acelerómetro
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;   // inicio bloque 14 bytes raw
static const uint8_t REG_WHO_AM_I     = 0x75;   // identidad del chip (0x71)


// ═══════════════════════════════════════════════════════
// ESTADO DE COMANDOS (desde Python)
// ═══════════════════════════════════════════════════════
// Formato del comando que Python envía por USB:
//   CMD:motor_izq,motor_der,aux_pwm,aux_enable,relay_open
// Ejemplo: "CMD:80,80,200,1,0\n" (motores a +80, bomba/cepillo PWM=200
// activados, relé CERRADO = compresor ON).

struct CommandState {
  int16_t motorIzq   = 0;      // -255..255, signo = sentido
  int16_t motorDer   = 0;
  uint8_t auxPwm     = 0;      // 0..255, PWM de bomba/cepillo
  bool    bombaEnable = false; // true = habilita bomba
  bool    cepEnable   = false; // true = habilita cepillos
  bool    relayOpen   = false; // true = corta compresor (emergencia)
};

CommandState cmd;              // estado actual de comandos aplicados
uint32_t lastCmdMillis = 0;    // último instante en que llegó un CMD: válido


// ═══════════════════════════════════════════════════════
// ESTADO DEL GIROSCOPIO
// ═══════════════════════════════════════════════════════

// Bias del gyro en el eje de yaw. Se calcula al inicio con el
// robot quieto y se resta de cada lectura.
float gyroBias_dps = 0.0f;

// Tasa angular filtrada con EMA (grados/segundo).
float gyroFiltered_dps = 0.0f;

// Acumulador de cambio de yaw pendiente por reportar.
// Se envía como "DANG:<valor>" cuando supera MIN_DELTA_TO_SEND_DEG.
float pendingDeltaYawDeg = 0.0f;

// Timers internos (micros para integración precisa, millis
// para el resto)
uint32_t lastGyroMicros      = 0;   // último micros() de la lectura
uint32_t lastTelemetryMillis = 0;   // último millis() del envío DANG:
uint32_t lastGyroReadMs      = 0;   // último millis() de lectura real


// Buffer de caracteres recibidos por Serial hasta ver '\n'
String rxLine;


// ═══════════════════════════════════════════════════════
// ESTADO DEL IMU (MPU9250)
// ═══════════════════════════════════════════════════════
// Si la lectura del MPU falla muchas veces seguidas, se desactiva
// temporalmente para que el resto del robot siga funcionando.
// Se reintenta después de IMU_RETRY_MS.

static bool     imuEnabled    = true;
static uint8_t  imuFailCount  = 0;
static const uint8_t  IMU_MAX_FAILS = 5;
static const uint32_t IMU_RETRY_MS  = 1000;
static uint32_t imuRetryAtMs  = 0;


// Estructura para guardar una muestra cruda del MPU
struct ImuSample {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
};


// ═══════════════════════════════════════════════════════
// UTILIDADES I2C GENERALES
// ═══════════════════════════════════════════════════════

// Vacía cualquier byte pendiente en el buffer RX de Wire.
// Se llama cuando una transacción I2C falla para no dejar
// basura que confunda a la siguiente lectura.
void flushWireRxBuffer() {
  while (Wire.available()) (void)Wire.read();
}


// ═══════════════════════════════════════════════════════
// UTILIDADES I2C DEL MPU9250
// ═══════════════════════════════════════════════════════

// Escribe un byte en un registro del MPU. Retorna true si OK.
bool mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Lee N bytes consecutivos a partir de un registro.
// dest debe apuntar a un buffer con al menos count bytes.
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

// Lee el registro WHO_AM_I (debería retornar 0x71 para MPU9250).
// Sirve para comprobar que el chip responde al arranque.
bool mpuReadWhoAmI(uint8_t &whoami) {
  uint8_t data = 0;
  if (!mpuReadBytes(REG_WHO_AM_I, 1, &data)) return false;
  whoami = data;
  return true;
}

// Lee una muestra completa (acelerómetro + gyro) en 14 bytes
// consecutivos. Los bytes vienen en orden: AX, AY, AZ, Temp, GX, GY, GZ
// (cada uno en 2 bytes big-endian).
bool mpuReadImuSample(ImuSample &s) {
  uint8_t raw[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, 14, raw)) return false;

  s.accelRaw[0] = (int16_t)((raw[0]  << 8) | raw[1]);
  s.accelRaw[1] = (int16_t)((raw[2]  << 8) | raw[3]);
  s.accelRaw[2] = (int16_t)((raw[4]  << 8) | raw[5]);
  // raw[6] y raw[7] son temperatura (ignorados)
  s.gyroRaw[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
  s.gyroRaw[1]  = (int16_t)((raw[10] << 8) | raw[11]);
  s.gyroRaw[2]  = (int16_t)((raw[12] << 8) | raw[13]);
  return true;
}

// Convierte el valor raw del giroscopio (int16) a grados/segundo.
// Con el rango ±250 dps, la escala es 131 LSB/dps.
float rawGyroToDps(int16_t raw) {
  return ((float)raw) / 131.0f;
}

// Configura el MPU9250 al arrancar:
// - Sale del modo sleep
// - Configura filtro digital (DLPF) a ~98Hz
// - Rango del gyro = ±250 dps (máxima sensibilidad)
// - Rango del accel = ±2g
bool initMPU9250() {
  delay(100);
  if (!mpuWriteByte(REG_PWR_MGMT_1,   0x00)) return false;   // wake
  delay(100);
  if (!mpuWriteByte(REG_CONFIG,       0x03)) return false;   // DLPF
  if (!mpuWriteByte(REG_GYRO_CONFIG,  0x00)) return false;   // ±250 dps
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;   // ±2g
  delay(50);
  return true;
}


// ═══════════════════════════════════════════════════════
// CALIBRACIÓN DEL GIROSCOPIO
// ═══════════════════════════════════════════════════════
// Toma CALIB_SAMPLES muestras con el robot quieto y calcula
// el bias promedio del eje de yaw. Este bias se restará de
// cada lectura futura para eliminar el offset constante del chip.
//
// Bloquea ~1 segundo durante el setup.

void calibrateGyro() {
  Serial.println("INFO:Calibrando gyro, deja el robot quieto...");
  float sum = 0.0f;
  uint16_t ok = 0;

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

  Serial.print("INFO:Gyro bias yaw = ");
  Serial.println(gyroBias_dps, 6);
}


// ═══════════════════════════════════════════════════════
// CONTROL DE UN BTS7960
// ═══════════════════════════════════════════════════════
// Maneja los 4 pines del driver para aplicar un PWM con signo:
//   value > 0 → gira en un sentido, pwm en RPWM
//   value < 0 → gira en sentido contrario, pwm en LPWM
//   value = 0 → ambos PWM en 0 (freno pasivo)
//
// Los enables (R_EN, L_EN) se dejan siempre HIGH mientras
// esté energizado el driver.

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
  if (value > 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, pwm);
  }
}

// Detiene un driver BTS7960 (enables siguen HIGH, PWM a 0).
void stopBTS7960(uint8_t r_en, uint8_t l_en, uint8_t rpwm, uint8_t lpwm) {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);
}


// ═══════════════════════════════════════════════════════
// CONTROL DE AUXILIARES (BOMBA + CEPILLOS)
// ═══════════════════════════════════════════════════════
// Llama explícitamente a los dos pines PWM aunque compartan
// número físico (pin 12). Así, si algún día separas el cableado,
// solo hay que cambiar CEP_PWM sin tocar más lógica.

void driveAuxiliaries(uint8_t pwmValue, bool bombaEnable, bool cepEnable) {
  digitalWrite(BOMBA_ENAL, bombaEnable ? HIGH : LOW);
  digitalWrite(BOMBA_ENAR, bombaEnable ? HIGH : LOW);
  digitalWrite(CEP_ENAL,   cepEnable   ? HIGH : LOW);
  digitalWrite(CEP_ENAR,   cepEnable   ? HIGH : LOW);

  // BOMBA_PWM y CEP_PWM apuntan al mismo pin físico (D12).
  // No se pueden mandar PWMs distintos; el enable individual decide
  // cuál carga recibe el PWM compartido.
  if (pwmValue == 0 || (!bombaEnable && !cepEnable)) {
    analogWrite(BOMBA_PWM, 0);
    return;
  }

  analogWrite(BOMBA_PWM, pwmValue);
}


// ═══════════════════════════════════════════════════════
// CONTROL DEL RELÉ DEL COMPRESOR
// ═══════════════════════════════════════════════════════
// Aplica el estado relayOpen del comando al pin D24.
//   relayOpen=true  → D24 en RELAY_OPEN_LEVEL (relé ABIERTO, corta compresor)
//   relayOpen=false → D24 en RELAY_CLOSED_LEVEL (relé CERRADO, compresor ON)

void applyRelay(bool openFlag) {
  digitalWrite(RELAY_COMPRESOR_PIN,
               openFlag ? RELAY_OPEN_LEVEL : RELAY_CLOSED_LEVEL);
}


// ═══════════════════════════════════════════════════════
// APLICAR / DETENER TODAS LAS SALIDAS
// ═══════════════════════════════════════════════════════

// Ejecuta todo el estado actual en el hardware.
// Se llama cada vez que se recibe un CMD: nuevo.
void applyOutputs() {
  driveBTS7960(cmd.motorIzq, MIZQ_ENAR, MIZQ_ENAL, MIZQ_PWMR, MIZQ_PWML);
  driveBTS7960(cmd.motorDer, MDER_ENAR, MDER_ENAL, MDER_PWMR, MDER_PWML);
  driveAuxiliaries(cmd.auxPwm, cmd.bombaEnable, cmd.cepEnable);
  applyRelay(cmd.relayOpen);
}

// Detiene motores y auxiliares. El relé NO se fuerza abierto
// aquí; si Python quiere corte de emergencia, debe mandarlo
// explícitamente.
void stopAllOutputs() {
  cmd.motorIzq   = 0;
  cmd.motorDer   = 0;
  cmd.auxPwm      = 0;
  cmd.bombaEnable = false;
  cmd.cepEnable   = false;
  // cmd.relayOpen se conserva para no sobrescribir emergencias
  applyOutputs();
}


// ═══════════════════════════════════════════════════════
// PARSER DE COMANDOS SERIALES
// ═══════════════════════════════════════════════════════
// Formato de entrada (una línea terminada en \n):
//   CMD:motor_izq,motor_der,aux_pwm,bomba_enable,cep_enable,relay_open
//
// Ejemplo:
//   CMD:80,-80,200,1,1,0
//     → motor izquierdo +80, derecho -80 (gira sobre sí)
//     → bomba y cepillos activos al PWM=200
//     → relé cerrado (compresor ON)
//
// Retrocompatibilidad:
//   - 4 campos: aux_enable único, sin relay_open
//   - 5 campos: aux_enable único + relay_open
//   - 6 campos: bomba_enable, cep_enable, relay_open

bool parseCommandLine(const String &line, CommandState &out) {
  if (!line.startsWith("CMD:")) return false;

  String payload = line.substring(4);

  int p1 = payload.indexOf(',');       if (p1 < 0) return false;
  int p2 = payload.indexOf(',', p1+1); if (p2 < 0) return false;
  int p3 = payload.indexOf(',', p2+1); if (p3 < 0) return false;
  int p4 = payload.indexOf(',', p3+1);
  int p5 = (p4 >= 0) ? payload.indexOf(',', p4+1) : -1;

  out.motorIzq = constrain(payload.substring(0, p1).toInt(), -255, 255);
  out.motorDer = constrain(payload.substring(p1 + 1, p2).toInt(), -255, 255);
  out.auxPwm   = (uint8_t)constrain(payload.substring(p2 + 1, p3).toInt(), 0, 255);

  if (p4 < 0) {
    bool auxEnable = (payload.substring(p3 + 1).toInt() != 0);
    out.bombaEnable = auxEnable;
    out.cepEnable   = auxEnable;
    out.relayOpen   = false;
  } else if (p5 < 0) {
    bool auxEnable = (payload.substring(p3 + 1, p4).toInt() != 0);
    out.bombaEnable = auxEnable;
    out.cepEnable   = auxEnable;
    out.relayOpen   = (payload.substring(p4 + 1).toInt() != 0);
  } else {
    out.bombaEnable = (payload.substring(p3 + 1, p4).toInt() != 0);
    out.cepEnable   = (payload.substring(p4 + 1, p5).toInt() != 0);
    out.relayOpen   = (payload.substring(p5 + 1).toInt() != 0);
  }

  return true;
}


// ═══════════════════════════════════════════════════════
// ACK (depuración) — confirma a Python qué se recibió
// ═══════════════════════════════════════════════════════
// Formato: ACK:izq,der,aux,bomba_enable,cep_enable,relay_open,bomba_EL,cep_EL
// Los dos últimos son el nivel real del pin enable leído de vuelta
// del hardware, útil para detectar fallos de cableado.

void sendAck() {
  if (!ACK_COMMANDS) return;
  Serial.print("ACK:");
  Serial.print(cmd.motorIzq);              Serial.print(",");
  Serial.print(cmd.motorDer);              Serial.print(",");
  Serial.print(cmd.auxPwm);                  Serial.print(",");
  Serial.print(cmd.bombaEnable ? 1 : 0);      Serial.print(",");
  Serial.print(cmd.cepEnable ? 1 : 0);        Serial.print(",");
  Serial.print(cmd.relayOpen ? 1 : 0);        Serial.print(",");
  Serial.print(digitalRead(BOMBA_ENAL));   Serial.print(",");
  Serial.println(digitalRead(CEP_ENAL));
}


// ═══════════════════════════════════════════════════════
// MANEJO DEL PUERTO SERIAL
// ═══════════════════════════════════════════════════════
// Lee byte a byte. Cuando ve '\n' procesa la línea completa.
// Limita el largo del buffer a 80 chars para prevenir overflow
// si llega basura.

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


// ═══════════════════════════════════════════════════════
// WATCHDOG — para el robot si Python se queda mudo
// ═══════════════════════════════════════════════════════
// Si no llegan CMD: por más de CMD_TIMEOUT_MS, detiene motores
// y auxiliares. El relé NO se fuerza aquí para no interferir
// con la lógica de emergencia de Python.

void safetyStopIfNoCommand() {
  if ((millis() - lastCmdMillis) > CMD_TIMEOUT_MS) {
    if (cmd.motorIzq != 0 || cmd.motorDer != 0 ||
        cmd.auxPwm != 0 || cmd.bombaEnable || cmd.cepEnable) {
      stopAllOutputs();
    }
  }
}


// ═══════════════════════════════════════════════════════
// LOOP DE GIROSCOPIO — lee y envía DANG: al Python
// ═══════════════════════════════════════════════════════
// Se llama cada iteración de loop(). Internamente respeta
// GYRO_READ_MS para no leer más rápido que lo necesario.
//
// Flujo:
//   1. Si el IMU estaba desactivado, intenta reactivar.
//   2. Lee una muestra cruda del MPU.
//   3. Convierte a dps, resta bias, aplica GYRO_SIGN y filtra.
//   4. Multiplica por dt para obtener delta-yaw en grados.
//   5. Acumula en pendingDeltaYawDeg.
//   6. Envía "DANG:<valor>\n" cuando:
//      - el acumulado supera MIN_DELTA_TO_SEND_DEG, o
//      - han pasado MAX_SILENCE_MS sin enviar nada y hay algo pendiente.

void updateGyroAndSendDelta() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // ─── Reactivación tras fallo ───
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

  // ─── Limitar frecuencia de lectura ───
  if ((nowMs - lastGyroReadMs) < GYRO_READ_MS) return;
  lastGyroReadMs = nowMs;

  // Primera vez: inicializar timestamps y salir
  if (lastGyroMicros == 0) {
    lastGyroMicros = nowUs;
    lastTelemetryMillis = nowMs;
    return;
  }

  // ─── Leer muestra cruda ───
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

  // ─── Calcular dt en segundos ───
  float dt = (nowUs - lastGyroMicros) * 1e-6f;
  lastGyroMicros = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return;   // ignora saltos raros

  // ─── Convertir a dps, restar bias, aplicar signo ───
  float yawRateRaw_dps = rawGyroToDps(s.gyroRaw[YAW_GYRO_AXIS]);
  float yawRate_dps    = (yawRateRaw_dps - gyroBias_dps) * GYRO_SIGN;

  // ─── Filtro EMA + dead-band ───
  gyroFiltered_dps = (GYRO_ALPHA * yawRate_dps) +
                     ((1.0f - GYRO_ALPHA) * gyroFiltered_dps);

  if (fabs(gyroFiltered_dps) < GYRO_DEADBAND_DPS) {
    gyroFiltered_dps = 0.0f;
  }

  // ─── Integrar a delta de yaw ───
  float deltaYawDeg = gyroFiltered_dps * dt;
  pendingDeltaYawDeg += deltaYawDeg;

  // ─── Enviar al Python si corresponde ───
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


// ═══════════════════════════════════════════════════════
// SETUP — se ejecuta una vez al arrancar el Arduino
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(SERIAL_BAUD);

  // ─── Pines de motores de tracción ───
  pinMode(MIZQ_ENAR, OUTPUT);  pinMode(MIZQ_ENAL, OUTPUT);
  pinMode(MIZQ_PWMR, OUTPUT);  pinMode(MIZQ_PWML, OUTPUT);
  pinMode(MDER_ENAR, OUTPUT);  pinMode(MDER_ENAL, OUTPUT);
  pinMode(MDER_PWMR, OUTPUT);  pinMode(MDER_PWML, OUTPUT);

  // ─── Pines de bomba / cepillos ───
  pinMode(BOMBA_ENAL, OUTPUT); pinMode(BOMBA_ENAR, OUTPUT);
  pinMode(BOMBA_PWM,  OUTPUT);
  pinMode(CEP_ENAL,   OUTPUT); pinMode(CEP_ENAR,   OUTPUT);
  pinMode(CEP_PWM,    OUTPUT);

  // ─── Pin del relé de emergencia ───
  pinMode(RELAY_COMPRESOR_PIN, OUTPUT);
  // Estado inicial = relé CERRADO (compresor alimentado por defecto)
  digitalWrite(RELAY_COMPRESOR_PIN, RELAY_CLOSED_LEVEL);

  // ─── Enables HIGH de inicio (BTS7960 los requiere) ───
  digitalWrite(MIZQ_ENAR, HIGH);   digitalWrite(MIZQ_ENAL, HIGH);
  digitalWrite(MDER_ENAR, HIGH);   digitalWrite(MDER_ENAL, HIGH);
  digitalWrite(BOMBA_ENAL, HIGH);  digitalWrite(BOMBA_ENAR, HIGH);
  digitalWrite(CEP_ENAL,   HIGH);  digitalWrite(CEP_ENAR,   HIGH);

  // Forzar PWM a 0 en todos los outputs
  stopAllOutputs();

  // ─── Bus I2C ───
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  delay(300);

  // ─── Inicializar MPU9250 ───
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

  // ─── Timers ───
  lastGyroMicros      = micros();
  lastTelemetryMillis = millis();
  lastCmdMillis       = millis();
  lastGyroReadMs      = 0;

  Serial.println("INFO:Sistema listo");

  // ─── Diagnóstico: reportar estado de enables auxiliares + relé ───
  Serial.print("INFO:ENABLES bomba_EL="); Serial.print(digitalRead(BOMBA_ENAL));
  Serial.print(" bomba_ER=");             Serial.print(digitalRead(BOMBA_ENAR));
  Serial.print(" cep_EL=");               Serial.print(digitalRead(CEP_ENAL));
  Serial.print(" cep_ER=");               Serial.print(digitalRead(CEP_ENAR));
  Serial.print(" relay_D24=");            Serial.println(digitalRead(RELAY_COMPRESOR_PIN));
}


// ═══════════════════════════════════════════════════════
// LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════
// Orden de prioridad:
//   1. Procesar comandos entrantes de Python (handleSerial)
//   2. Ejecutar watchdog de seguridad (safetyStopIfNoCommand)
//   3. Actualizar sensores y enviar telemetría (gyro)
//
// handleSerial se llama dos veces para procesar cualquier
// comando que haya llegado mientras leíamos el gyro.

void loop() {
  handleSerial();
  safetyStopIfNoCommand();

  updateGyroAndSendDelta();

  handleSerial();
  safetyStopIfNoCommand();
}


// ═══════════════════════════════════════════════════════
// CÓDIGO ANTIGUO DE ENCODERS AS5600 + MULTIPLEXOR TCA9548A
// ═══════════════════════════════════════════════════════
// DESACTIVADO — este bloque está comentado. Si en el futuro
// quieres volver a activar los encoders, descomenta todo el
// bloque de abajo, agrega las llamadas updateEncoders() y
// reportEncoders() dentro de loop(), y maneja los mensajes
// "ENC:..." en el lado de Python.
//
// Requiere un TCA9548A (multiplexor I2C) con:
//   - canal 6 conectado al AS5600 izquierdo
//   - canal 7 conectado al AS5600 derecho
// Y los pines DIR del AS5600 en Arduino: D7 (izq), D6 (der)
//
/*
// PINES DIR DEL AS5600 — configuran sentido de conteo
static const uint8_t ENC_DIR_IZQ = 7;
static const uint8_t ENC_DIR_DER = 6;
static const uint8_t ENC_DIR_IZQ_LEVEL = LOW;
static const uint8_t ENC_DIR_DER_LEVEL = HIGH;

// MULTIPLEXOR I2C (TCA9548A / HW617)
static const uint8_t TCA_ADDR   = 0x70;
static const uint8_t TCA_CH_IZQ = 6;
static const uint8_t TCA_CH_DER = 7;

// CONFIG DE LECTURA AS5600
static const uint8_t  AS5600_ADDR        = 0x36;
static const uint8_t  AS5600_RAW_ANGLE_H = 0x0C;
static const uint8_t  AS5600_RAW_ANGLE_L = 0x0D;
static const uint16_t AS5600_MAX_RAW     = 4096;
static const uint32_t ENC_REPORT_MS      = 20;
static const uint32_t ENC_READ_MS        = 20;
static const uint8_t  ENC_MAX_FAILS      = 3;
static const uint32_t ENC_RETRY_MS       = 1000;

// ESTADO DE ENCODERS
static uint16_t encRawPrev_Izq = 0;
static uint16_t encRawPrev_Der = 0;
static float    encAccumDeg_Izq = 0.0f;
static float    encAccumDeg_Der = 0.0f;
static float    encDeltaDeg_Izq = 0.0f;
static float    encDeltaDeg_Der = 0.0f;
static uint32_t lastEncReportMs = 0;
static uint32_t lastEncReadMs   = 0;
static bool     encInitialized  = false;
static bool     encodersEnabled = true;
static uint8_t  encFailCountIzq = 0;
static uint8_t  encFailCountDer = 0;
static uint32_t encRetryAtMs    = 0;

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

void updateEncoders() {
  uint32_t nowMs = millis();
  if (!encodersEnabled) {
    if (nowMs < encRetryAtMs) return;
    encodersEnabled = true;
    encFailCountIzq = 0;
    encFailCountDer = 0;
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

// En setup(), poner antes de Wire.begin():
//   pinMode(ENC_DIR_IZQ, OUTPUT);
//   pinMode(ENC_DIR_DER, OUTPUT);
//   digitalWrite(ENC_DIR_IZQ, ENC_DIR_IZQ_LEVEL);
//   digitalWrite(ENC_DIR_DER, ENC_DIR_DER_LEVEL);
//   delay(10);
// En loop(), agregar:
//   updateEncoders();
//   reportEncoders();
*/
