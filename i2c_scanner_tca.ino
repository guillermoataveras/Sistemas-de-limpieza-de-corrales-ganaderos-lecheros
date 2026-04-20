/*
 * i2c_scanner_tca.ino — Diagnóstico I2C + TCA9548A
 * ===================================================
 * Sube este sketch temporal a tu Arduino Mega.
 * Abre el Serial Monitor a 115200 baudios.
 *
 * Lo que verás:
 *   1. Todas las direcciones I2C que responden en el bus principal
 *      → Debes ver al menos: 0x68 (MPU9250) y 0x70 (TCA9548A)
 *   2. Si el TCA9548A responde, para cada uno de sus 8 canales,
 *      las direcciones que responden en ese canal
 *      → Debes ver 0x36 (AS5600) en el canal 0 y el canal 1
 *
 * Si NO ves 0x70 → el multiplexor está mal cableado o no tiene alimentación
 * Si ves 0x70 pero ningún 0x36 en ningún canal → los AS5600 no están conectados
 *                                                  o no tienen imán
 * Si ves 0x36 en un canal distinto → ajustar TCA_CH_IZQ / TCA_CH_DER en main.cpp
 */

#include <Arduino.h>
#include <Wire.h>

static const uint8_t TCA_ADDR = 0x70;   // dirección por defecto del TCA9548A

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

void scanBus(const char* label) {
  Serial.print("  [");
  Serial.print(label);
  Serial.print("] Direcciones encontradas: ");

  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      if (count > 0) Serial.print(", ");
      Serial.print("0x");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);
      count++;
    }
  }

  if (count == 0) Serial.print("(ninguna)");
  Serial.print("  [total=");
  Serial.print(count);
  Serial.println("]");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(500);

  Serial.println();
  Serial.println("=======================================================");
  Serial.println(" I2C SCANNER + TCA9548A DIAGNOSTIC");
  Serial.println("=======================================================");

  Wire.begin();
  Wire.setClock(100000);   // 100 kHz — conservador para diagnóstico
  delay(200);

  // ─── Paso 1: escanear bus principal ─────────────────────────
  Serial.println();
  Serial.println("Paso 1: Bus principal (todos los canales del TCA deshabilitados)");
  tcaDisableAll();
  delay(50);
  scanBus("BUS PRINCIPAL");

  // ─── Paso 2: verificar TCA9548A específicamente ─────────────
  Serial.println();
  Serial.println("Paso 2: Verificacion del TCA9548A en 0x70");
  Wire.beginTransmission(TCA_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("  TCA9548A detectado OK en 0x70");
  } else {
    Serial.print("  TCA9548A NO RESPONDE en 0x70 (error=");
    Serial.print(err);
    Serial.println(")");
    Serial.println("  Verifica:");
    Serial.println("   - VCC del TCA a 3.3V o 5V");
    Serial.println("   - GND comun con el Arduino");
    Serial.println("   - SDA del TCA al pin 20 del Mega");
    Serial.println("   - SCL del TCA al pin 21 del Mega");
    Serial.println("   - Pull-ups de 4.7k en SDA y SCL");
    Serial.println("   - Pines A0/A1/A2 del TCA a GND (direccion 0x70)");
    Serial.println();
    Serial.println("FIN — corrige el cableado del TCA y vuelve a probar.");
    return;
  }

  // ─── Paso 3: escanear cada canal del TCA ────────────────────
  Serial.println();
  Serial.println("Paso 3: Escaneo por canal del TCA9548A");
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaSelect(ch);
    delay(20);
    char buf[16];
    snprintf(buf, sizeof(buf), "CH%u", ch);
    scanBus(buf);
  }

  tcaDisableAll();

  // ─── Paso 4: test específico del AS5600 ─────────────────────
  Serial.println();
  Serial.println("Paso 4: Lectura directa del AS5600 en canales 0 y 1");
  for (uint8_t ch = 0; ch < 2; ch++) {
    Serial.print("  Canal ");
    Serial.print(ch);
    Serial.print(": ");

    tcaSelect(ch);
    delay(20);

    // Leer el raw angle del AS5600 (registros 0x0C/0x0D)
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) {
      Serial.println("NO RESPONDE a 0x36");
      continue;
    }
    uint8_t n = Wire.requestFrom((uint8_t)0x36, (uint8_t)2);
    if (n != 2) {
      Serial.println("no devuelve 2 bytes");
      continue;
    }
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    uint16_t raw = ((uint16_t)(hi & 0x0F) << 8) | lo;
    float deg = (float)raw * (360.0f / 4096.0f);

    Serial.print("raw=");
    Serial.print(raw);
    Serial.print("  angulo=");
    Serial.print(deg, 1);
    Serial.print("°");

    // Leer STATUS (registro 0x0B) — indica presencia y calidad del iman
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x36, (uint8_t)1);
    if (Wire.available()) {
      uint8_t st = Wire.read();
      bool md = st & (1 << 5);   // Magnet Detected
      bool ml = st & (1 << 4);   // Magnet too weak
      bool mh = st & (1 << 3);   // Magnet too strong
      Serial.print("  [");
      if (!md)       Serial.print("SIN IMAN");
      else if (ml)   Serial.print("IMAN LEJOS");
      else if (mh)   Serial.print("IMAN CERCA");
      else           Serial.print("IMAN OK");
      Serial.print("]");
    }
    Serial.println();
  }

  tcaDisableAll();

  Serial.println();
  Serial.println("=======================================================");
  Serial.println(" FIN DEL DIAGNOSTICO");
  Serial.println("=======================================================");
}

void loop() {
  // Nada — el diagnóstico corre solo una vez en setup().
}
