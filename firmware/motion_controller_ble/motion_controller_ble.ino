#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "ble_telemetry.h"

// ============================================================
// ESP32-C3 Super Mini + MPU-9250 + AK8963
// Motion Controller Lab — Version BLE + USB Serial
// SDA=GPIO8, SCL=GPIO9, MPU=0x68, AK8963=0x0C por bypass
// USB CDC On Boot: Enabled
// ============================================================

// -------------------- Hardware ------------------------------
static const uint8_t SDA_PIN = 8;
static const uint8_t SCL_PIN = 9;
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t AK8963_ADDR = 0x0C;

// -------------------- Registros MPU-9250 --------------------
static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static const uint8_t REG_INT_PIN_CFG   = 0x37;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_WHO_AM_I      = 0x75;

// -------------------- Registros AK8963 ----------------------
static const uint8_t AK_WIA   = 0x00;
static const uint8_t AK_ST1   = 0x02;
static const uint8_t AK_HXL   = 0x03;
static const uint8_t AK_CNTL1 = 0x0A;
static const uint8_t AK_ASAX  = 0x10;

// -------------------- Escalas IMU ---------------------------
static const float ACCEL_SCALE = 16384.0f;  // +/-2 g
static const float GYRO_SCALE  = 65.5f;     // +/-500 dps

// AK8963 en modo 16 bits: sensibilidad nominal aproximada 0.15 uT/LSB.
// Se aplicara tambien el ajuste ASA de fabrica.
static const float MAG_UT_PER_LSB = 0.15f;

// -------------------- Frecuencia ----------------------------
static const uint32_t SAMPLE_PERIOD_US = 5000; // 200 Hz IMU
static const uint32_t CSV_PERIOD_MS = 20;      // 50 filas/s
static const uint32_t MAG_PRINT_PERIOD_MS = 100;
static const uint32_t MAG_CAL_DURATION_MS = 30000; // 30 s de movimiento libre
static const uint32_t MAG_SAMPLE_TIMEOUT_MS = 50;  // Caducidad de frescura de muestra (nominal 10 ms a 100 Hz)

// -------------------- Madgwick IMU --------------------------
static const float MADGWICK_BETA = 0.08f;

// -------------------- Calibracion gyro ----------------------
static const int CALIBRATION_SAMPLES = 600;
static const uint32_t CALIBRATION_DELAY_MS = 5;

// -------------------- Estado -------------------------------
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float axG = 0.0f, ayG = 0.0f, azG = 1.0f;
float gxDps = 0.0f, gyDps = 0.0f, gzDps = 0.0f;

float magX_uT = 0.0f, magY_uT = 0.0f, magZ_uT = 0.0f;
int16_t magRawX = 0, magRawY = 0, magRawZ = 0;
float magAdjX = 1.0f, magAdjY = 1.0f, magAdjZ = 1.0f;
bool magDataReady = false;
uint32_t lastMagSampleMs = 0;
float magMinX = 1e6f, magMinY = 1e6f, magMinZ = 1e6f;
float magMaxX = -1e6f, magMaxY = -1e6f, magMaxZ = -1e6f;
float magOffsetX = 0.0f, magOffsetY = 0.0f, magOffsetZ = 0.0f;
float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;
bool magCalibrated = false;
bool magCoverageValid = false;
uint32_t magCalibrationSamples = 0;
float magSpanX = 0.0f, magSpanY = 0.0f, magSpanZ = 0.0f;
bool yawMagValid = false;
float magNormRaw = 0.0f, magNormCal = 0.0f;
float magXCal = 0.0f, magYCal = 0.0f, magZCal = 0.0f;
float yawMagDeg = 0.0f, yawMagRelativeDeg = 0.0f, yawMagReferenceDeg = 0.0f;
bool yawMagReferenceCaptured = false;

uint32_t i2cErrors = 0;
uint32_t magErrors = 0;
uint32_t lastCsvMs = 0;
uint32_t lastMagPrintMs = 0;
uint32_t lastSampleUs = 0;

// Quaternion de orientacion: w, x, y, z
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

// Quaternion de referencia
float refW = 1.0f, refX = 0.0f, refY = 0.0f, refZ = 0.0f;
bool referenceCaptured = false;

float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;

// -------------------- Orientacion de Montaje & NVS ----------
#include <Preferences.h>
Preferences nvsPrefs;

float mountYawDeg = 0.0f;
float mountPitchDeg = 0.0f;
float mountRollDeg = 0.0f;
float R_mount[3][3] = {
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 1.0f}
};
volatile bool s_bleCalReceived = false;

void computeMountingMatrix() {
  float psi = mountYawDeg * DEG_TO_RAD;
  float theta = mountPitchDeg * DEG_TO_RAD;
  float phi = mountRollDeg * DEG_TO_RAD;

  float c_psi = cosf(psi), s_psi = sinf(psi);
  float c_th = cosf(theta), s_th = sinf(theta);
  float c_ph = cosf(phi), s_ph = sinf(phi);

  // R = Rz(psi) * Ry(theta) * Rx(phi)
  R_mount[0][0] = c_psi * c_th;
  R_mount[0][1] = c_psi * s_th * s_ph - s_psi * c_ph;
  R_mount[0][2] = c_psi * s_th * c_ph + s_psi * s_ph;

  R_mount[1][0] = s_psi * c_th;
  R_mount[1][1] = s_psi * s_th * s_ph + c_psi * c_ph;
  R_mount[1][2] = s_psi * s_th * c_ph - c_psi * s_ph;

  R_mount[2][0] = -s_th;
  R_mount[2][1] = c_th * s_ph;
  R_mount[2][2] = c_th * c_ph;
}

void applyMountingRotation(float& x, float& y, float& z) {
  float rx = R_mount[0][0] * x + R_mount[0][1] * y + R_mount[0][2] * z;
  float ry = R_mount[1][0] * x + R_mount[1][1] * y + R_mount[1][2] * z;
  float rz = R_mount[2][0] * x + R_mount[2][1] * y + R_mount[2][2] * z;
  x = rx;
  y = ry;
  z = rz;
}

void getCalibratedMagBody(float& mx, float& my, float& mz) {
  // Remapeo de fábrica de ejes AK8963 al marco físico del sensor MPU:
  // X_mpu = Y_mag, Y_mpu = X_mag, Z_mpu = -Z_mag
  mx = (magY_uT - magOffsetY) * magScaleY;
  my = (magX_uT - magOffsetX) * magScaleX;
  mz = -((magZ_uT - magOffsetZ) * magScaleZ);
  // Transformación por matriz de rotación de montaje:
  applyMountingRotation(mx, my, mz);
}

void loadCalibrationFromNVS() {
  if (nvsPrefs.begin("mcl_cal", true)) {
    bool valid = nvsPrefs.getBool("valid", false);
    if (valid) {
      magOffsetX = nvsPrefs.getFloat("offX", 0.0f);
      magOffsetY = nvsPrefs.getFloat("offY", 0.0f);
      magOffsetZ = nvsPrefs.getFloat("offZ", 0.0f);
      magScaleX  = nvsPrefs.getFloat("scaleX", 1.0f);
      magScaleY  = nvsPrefs.getFloat("scaleY", 1.0f);
      magScaleZ  = nvsPrefs.getFloat("scaleZ", 1.0f);
      mountYawDeg   = nvsPrefs.getFloat("mYaw", 0.0f);
      mountPitchDeg = nvsPrefs.getFloat("mPitch", 0.0f);
      mountRollDeg  = nvsPrefs.getFloat("mRoll", 0.0f);
      computeMountingMatrix();
      magCalibrated = true;
      magCoverageValid = true;
      Serial.println("# NVS: Calibracion y orientacion cargadas de flash.");
      Serial.printf("# NVS: Mount Yaw=%.1f Pitch=%.1f Roll=%.1f\n", mountYawDeg, mountPitchDeg, mountRollDeg);
    }
    nvsPrefs.end();
  }
}

void saveCalibrationToNVS() {
  if (nvsPrefs.begin("mcl_cal", false)) {
    nvsPrefs.putBool("valid", true);
    nvsPrefs.putFloat("offX", magOffsetX);
    nvsPrefs.putFloat("offY", magOffsetY);
    nvsPrefs.putFloat("offZ", magOffsetZ);
    nvsPrefs.putFloat("scaleX", magScaleX);
    nvsPrefs.putFloat("scaleY", magScaleY);
    nvsPrefs.putFloat("scaleZ", magScaleZ);
    nvsPrefs.putFloat("mYaw", mountYawDeg);
    nvsPrefs.putFloat("mPitch", mountPitchDeg);
    nvsPrefs.putFloat("mRoll", mountRollDeg);
    nvsPrefs.end();
    Serial.println("# NVS: Calibracion y montaje guardados en flash.");
  }
}

void onBleCalibrationReceived(const BleCalibrationPacket& pkt) {
  if (pkt.version != 1) return;
  mountYawDeg = pkt.mountYawDeg10 / 10.0f;
  mountPitchDeg = pkt.mountPitchDeg10 / 10.0f;
  mountRollDeg = pkt.mountRollDeg10 / 10.0f;
  computeMountingMatrix();

  if (pkt.flags & 0x01) { // mag_valid
    magOffsetX = pkt.magOffsetX;
    magOffsetY = pkt.magOffsetY;
    magOffsetZ = pkt.magOffsetZ;
    magScaleX = pkt.magScaleX;
    magScaleY = pkt.magScaleY;
    magScaleZ = pkt.magScaleZ;
    magCalibrated = true;
    magCoverageValid = true;
  }

  if (pkt.flags & 0x04) { // save_nvs
    saveCalibrationToNVS();
  }

  s_bleCalReceived = true;
  Serial.printf("# BLE_CAL_RECEIVED: Mount Yaw=%.1f, Pitch=%.1f, Roll=%.1f\n",
                mountYawDeg, mountPitchDeg, mountRollDeg);
  Serial.printf("# BLE_CAL_RECEIVED: Offsets=(%.2f, %.2f, %.2f) Scales=(%.3f, %.3f, %.3f)\n",
                magOffsetX, magOffsetY, magOffsetZ, magScaleX, magScaleY, magScaleZ);

  // Actualiza la referencia para que el nuevo sistema de coordenadas sea el punto cero
  captureReference();
}

// -------------------- I2C MPU -------------------------------

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t received = Wire.requestFrom((uint8_t)MPU_ADDR, length, (uint8_t)true);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (!Wire.available()) return false;
    buffer[i] = Wire.read();
  }
  return true;
}

uint8_t readWhoAmI() {
  uint8_t value = 0;
  if (!readRegisters(REG_WHO_AM_I, &value, 1)) return 0;
  return value;
}

// -------------------- I2C AK8963 por bypass -----------------
bool writeMagRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMagRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t received = Wire.requestFrom((uint8_t)AK8963_ADDR, length, (uint8_t)true);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (!Wire.available()) return false;
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMPU() {
  Serial.println("# Inicializando MPU-9250...");

  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(100);

  if (!writeRegister(REG_CONFIG, 0x03)) return false;
  if (!writeRegister(REG_SMPLRT_DIV, 0x04)) return false;
  if (!writeRegister(REG_GYRO_CONFIG, 0x08)) return false;
  if (!writeRegister(REG_ACCEL_CONFIG, 0x00)) return false;
  if (!writeRegister(REG_ACCEL_CONFIG2, 0x03)) return false;
  delay(50);

  uint8_t who = readWhoAmI();
  Serial.print("# MPU WHO_AM_I=0x");
  Serial.println(who, HEX);

  if (who != 0x71 && who != 0x73) {
    Serial.println("# ERROR: identificacion MPU no valida.");
    return false;
  }

  Serial.println("# Configuracion MPU aplicada.");
  return true;
}

bool initMagnetometer() {
  Serial.println("# Inicializando AK8963...");

  // Bypass I2C: INT_PIN_CFG.BYPASS_EN = 1.
  if (!writeRegister(REG_INT_PIN_CFG, 0x02)) {
    Serial.println("# ERROR: no se pudo habilitar bypass.");
    return false;
  }
  delay(10);

  uint8_t id = 0;
  if (!readMagRegisters(AK_WIA, &id, 1)) {
    Serial.println("# ERROR: AK8963 no responde por I2C.");
    return false;
  }

  Serial.print("# AK8963 WIA=0x");
  Serial.println(id, HEX);
  if (id != 0x48) {
    Serial.println("# ERROR: identificador AK8963 inesperado.");
    return false;
  }

  // Power-down y luego Fuse ROM para obtener ajustes ASA.
  if (!writeMagRegister(AK_CNTL1, 0x00)) return false;
  delay(10);

  if (!writeMagRegister(AK_CNTL1, 0x0F)) return false;
  delay(10);

  uint8_t asa[3];
  if (!readMagRegisters(AK_ASAX, asa, 3)) {
    writeMagRegister(AK_CNTL1, 0x00);
    return false;
  }

  magAdjX = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
  magAdjY = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
  magAdjZ = ((float)(asa[2] - 128) / 256.0f) + 1.0f;

  Serial.print("# ASA factor X: "); Serial.println(magAdjX, 5);
  Serial.print("# ASA factor Y: "); Serial.println(magAdjY, 5);
  Serial.print("# ASA factor Z: "); Serial.println(magAdjZ, 5);

  if (!writeMagRegister(AK_CNTL1, 0x00)) return false;
  delay(10);

  // Continuous measurement mode 2, 16-bit output, 100 Hz.
  if (!writeMagRegister(AK_CNTL1, 0x16)) return false;
  delay(10);

  Serial.println("# AK8963 listo: modo continuo 100 Hz, 16 bits.");
  Serial.println("# NOTA: datos aun sin calibracion hard-iron/soft-iron.");
  return true;
}

bool readMagnetometer() {
  uint8_t st1 = 0;
  if (!readMagRegisters(AK_ST1, &st1, 1)) {
    magErrors++;
    return false;
  }

  // DRDY bit (bit 0 de ST1): 0 indica que no hay nueva muestra disponible
  if ((st1 & 0x01) == 0) return false;

  // Leer HXL..ST2 (7 bytes). Leer ST2 finaliza la lectura y desbloquea el sensor.
  uint8_t data[7];
  if (!readMagRegisters(AK_HXL, data, sizeof(data))) {
    magErrors++;
    return false;
  }

  // HOFL bit (bit 3 de ST2): saturación magnética (overflow)
  if (data[6] & 0x08) {
    magErrors++;
    return false;
  }

  magRawX = (int16_t)((uint16_t)data[1] << 8 | data[0]);
  magRawY = (int16_t)((uint16_t)data[3] << 8 | data[2]);
  magRawZ = (int16_t)((uint16_t)data[5] << 8 | data[4]);

  float candX = magRawX * MAG_UT_PER_LSB * magAdjX;
  float candY = magRawY * MAG_UT_PER_LSB * magAdjY;
  float candZ = magRawZ * MAG_UT_PER_LSB * magAdjZ;

  // Rechazo de datos no finitos (NaN / Inf)
  if (!isfinite(candX) || !isfinite(candY) || !isfinite(candZ)) {
    magErrors++;
    return false;
  }

  magX_uT = candX;
  magY_uT = candY;
  magZ_uT = candZ;
  magDataReady = true;
  lastMagSampleMs = millis();

  if (!magCalibrated) {
    magCalibrationSamples++;
    if (magX_uT < magMinX) magMinX = magX_uT; if (magX_uT > magMaxX) magMaxX = magX_uT;
    if (magY_uT < magMinY) magMinY = magY_uT; if (magY_uT > magMaxY) magMaxY = magY_uT;
    if (magZ_uT < magMinZ) magMinZ = magZ_uT; if (magZ_uT > magMaxZ) magMaxZ = magZ_uT;
  } else {
    magXCal = (magX_uT - magOffsetX) * magScaleX;
    magYCal = (magY_uT - magOffsetY) * magScaleY;
    magZCal = (magZ_uT - magOffsetZ) * magScaleZ;
  }
  return true;
}

// -------------------- Lectura IMU ---------------------------
bool readIMU() {
  uint8_t data[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
    i2cErrors++;
    return false;
  }

  int16_t rawAx = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  int16_t rawAy = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  int16_t rawAz = (int16_t)((uint16_t)data[4] << 8 | data[5]);
  int16_t rawGx = (int16_t)((uint16_t)data[8] << 8 | data[9]);
  int16_t rawGy = (int16_t)((uint16_t)data[10] << 8 | data[11]);
  int16_t rawGz = (int16_t)((uint16_t)data[12] << 8 | data[13]);

  axG = rawAx / ACCEL_SCALE;
  ayG = rawAy / ACCEL_SCALE;
  azG = rawAz / ACCEL_SCALE;

  gxDps = rawGx / GYRO_SCALE - gyroBiasX;
  gyDps = rawGy / GYRO_SCALE - gyroBiasY;
  gzDps = rawGz / GYRO_SCALE - gyroBiasZ;

  // Aplica matriz de rotación de montaje al marco body
  applyMountingRotation(axG, ayG, azG);
  applyMountingRotation(gxDps, gyDps, gzDps);
  return true;
}

bool calibrateGyro() {
  Serial.println("# CALIBRACION: no mover el sensor.");

  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int validSamples = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    uint8_t data[14];
    if (readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
      int16_t rawGx = (int16_t)((uint16_t)data[8] << 8 | data[9]);
      int16_t rawGy = (int16_t)((uint16_t)data[10] << 8 | data[11]);
      int16_t rawGz = (int16_t)((uint16_t)data[12] << 8 | data[13]);
      sumX += rawGx / GYRO_SCALE;
      sumY += rawGy / GYRO_SCALE;
      sumZ += rawGz / GYRO_SCALE;
      validSamples++;
    } else {
      i2cErrors++;
    }
    delay(CALIBRATION_DELAY_MS);
  }

  if (validSamples < CALIBRATION_SAMPLES * 0.9f) {
    Serial.println("# ERROR: demasiadas lecturas fallidas al calibrar.");
    return false;
  }

  gyroBiasX = sumX / validSamples;
  gyroBiasY = sumY / validSamples;
  gyroBiasZ = sumZ / validSamples;

  Serial.print("# Bias gyro X: "); Serial.println(gyroBiasX, 4);
  Serial.print("# Bias gyro Y: "); Serial.println(gyroBiasY, 4);
  Serial.print("# Bias gyro Z: "); Serial.println(gyroBiasZ, 4);
  return true;
}

// -------------------- Madgwick IMU (6 ejes) -----------------
// Implementación base con velocidades angulares en radianes por segundo (rad/s).
void madgwickUpdate6DOFRad(float gxRad, float gyRad, float gzRad,
                           float ax, float ay, float az, float dt) {
  float normA = sqrtf(ax * ax + ay * ay + az * az);
  float qDotW = 0.5f * (-qx * gxRad - qy * gyRad - qz * gzRad);
  float qDotX = 0.5f * ( qw * gxRad + qy * gzRad - qz * gyRad);
  float qDotY = 0.5f * ( qw * gyRad - qx * gzRad + qz * gxRad);
  float qDotZ = 0.5f * ( qw * gzRad + qx * gyRad - qy * gxRad);

  if (normA > 0.000001f) {
    ax /= normA;
    ay /= normA;
    az /= normA;

    float _2qw = 2.0f * qw;
    float _2qx = 2.0f * qx;
    float _2qy = 2.0f * qy;
    float _2qz = 2.0f * qz;
    float _4qw = 4.0f * qw;
    float _4qx = 4.0f * qx;
    float _4qy = 4.0f * qy;
    float _8qx = 8.0f * qx;
    float _8qy = 8.0f * qy;
    float qw2 = qw * qw;
    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;

    float sW = _4qw * qy2 + _2qy * ax + _4qw * qx2 - _2qx * ay;
    float sX = _4qx * qz2 - _2qz * ax + 4.0f * qw2 * qx -
               _2qw * ay - _4qx + _8qx * qx2 + _8qx * qy2 + _4qx * az;
    float sY = 4.0f * qw2 * qy + _2qw * ax + _4qy * qz2 -
               _2qz * ay - _4qy + _8qy * qx2 + _8qy * qy2 + _4qy * az;
    float sZ = 4.0f * qx2 * qz - _2qx * ax + 4.0f * qy2 * qz - _2qy * ay;

    float normS = sqrtf(sW*sW + sX*sX + sY*sY + sZ*sZ);
    if (normS > 0.000001f) {
      sW /= normS; sX /= normS; sY /= normS; sZ /= normS;
      qDotW -= MADGWICK_BETA * sW;
      qDotX -= MADGWICK_BETA * sX;
      qDotY -= MADGWICK_BETA * sY;
      qDotZ -= MADGWICK_BETA * sZ;
    }
  }

  qw += qDotW * dt;
  qx += qDotX * dt;
  qy += qDotY * dt;
  qz += qDotZ * dt;

  float normQ = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
  if (normQ > 0.000001f) {
    qw /= normQ; qx /= normQ; qy /= normQ; qz /= normQ;
  }
}

// Wrapper de 6 ejes que acepta velocidades angulares en grados por segundo (dps).
void madgwickUpdate6DOF(float gxDps, float gyDps, float gzDps,
                        float ax, float ay, float az, float dt) {
  madgwickUpdate6DOFRad(gxDps * 0.017453292519943295f,
                        gyDps * 0.017453292519943295f,
                        gzDps * 0.017453292519943295f,
                        ax, ay, az, dt);
}

// -------------------- Madgwick MARG (9 ejes) ----------------
// Fusión completa de acelerómetro, giróscopo y magnetómetro alineado.
// Elimina la deriva de Yaw proyectando el campo magnético en el plano horizontal.
void madgwickUpdate9DOF(float gxDps, float gyDps, float gzDps,
                        float ax, float ay, float az,
                        float mx, float my, float mz, float dt) {
  float gxRad = gxDps * 0.017453292519943295f;
  float gyRad = gyDps * 0.017453292519943295f;
  float gzRad = gzDps * 0.017453292519943295f;

  float normA = sqrtf(ax * ax + ay * ay + az * az);
  if (normA < 0.000001f) return;
  ax /= normA;
  ay /= normA;
  az /= normA;

  float normM = sqrtf(mx * mx + my * my + mz * mz);
  if (normM < 0.000001f) {
    // D-03: Invoca la función en radianes directamente para evitar la doble conversión deg->rad
    madgwickUpdate6DOFRad(gxRad, gyRad, gzRad, ax, ay, az, dt);
    return;
  }
  mx /= normM;
  my /= normM;
  mz /= normM;

  float q1 = qw, q2 = qx, q3 = qy, q4 = qz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Vector de referencia del campo magnético en el marco de la Tierra (D-04).
  // NOTA TÉCNICA (Nomenclatura canónica de Sebastian Madgwick, 2010):
  // hx, hy, hz representan el vector magnético rotado al marco de navegación (R^T * m).
  // Dado que |m|=1 y |q|=1, |h|=1. Por tanto, bx = sqrt(hx^2 + hy^2) y bz = hz tienen escala unitaria (1.0).
  // En la implementación de referencia original (MadgwickAHRS.c), las variables se denominan históricamente
  // '_2bx' y '_2bz', pero ambas almacenan los valores de escala 1.0 (bx y bz).
  // En las ecuaciones del gradiente (s1..s4), Madgwick utiliza '_4bx = 2 * _2bx' como factor 2*bx.
  // NO se debe añadir un factor de 2.0 a _2bx, pues alteraría la relación de aspecto del campo magnético (bx/bz).
  float hx = 2.0f * (mx * (0.5f - q3q3 - q4q4) + my * (q2q3 - q1q4) + mz * (q2q4 + q1q3));
  float hy = 2.0f * (mx * (q2q3 + q1q4) + my * (0.5f - q2q2 - q4q4) + mz * (q3q4 - q1q2));
  float _2bx = sqrtf(hx * hx + hy * hy);
  float _2bz = 2.0f * (mx * (q2q4 - q1q3) + my * (q3q4 + q1q2) + mz * (0.5f - q2q2 - q3q3));
  float _4bx = 2.0f * _2bx;
  float _4bz = 2.0f * _2bz;

  // Gradiente descendente combinado
  float s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay)
             - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

  float s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

  float s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

  float s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
             + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

  float normS = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  if (normS > 0.000001f) {
    s1 /= normS; s2 /= normS; s3 /= normS; s4 /= normS;
  } else {
    s1 = s2 = s3 = s4 = 0.0f;
  }

  float qDot1 = 0.5f * (-q2 * gxRad - q3 * gyRad - q4 * gzRad) - MADGWICK_BETA * s1;
  float qDot2 = 0.5f * ( q1 * gxRad + q3 * gzRad - q4 * gyRad) - MADGWICK_BETA * s2;
  float qDot3 = 0.5f * ( q1 * gyRad - q2 * gzRad + q4 * gxRad) - MADGWICK_BETA * s3;
  float qDot4 = 0.5f * ( q1 * gzRad + q2 * gyRad - q3 * gxRad) - MADGWICK_BETA * s4;

  qw += qDot1 * dt;
  qx += qDot2 * dt;
  qy += qDot3 * dt;
  qz += qDot4 * dt;

  float normQ = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (normQ > 0.000001f) {
    qw /= normQ; qx /= normQ; qy /= normQ; qz /= normQ;
  }
}

void captureReference() {
  refW = qw; refX = qx; refY = qy; refZ = qz;
  referenceCaptured = true;
}

void calculateRelativeAngles() {
  float rw = refW * qw + refX * qx + refY * qy + refZ * qz;
  float rx = refW * qx - refX * qw - refY * qz + refZ * qy;
  float ry = refW * qy + refX * qz - refY * qw - refZ * qx;
  float rz = refW * qz - refX * qy + refY * qx - refZ * qw;

  float normR = sqrtf(rw*rw + rx*rx + ry*ry + rz*rz);
  if (normR > 0.000001f) {
    rw /= normR; rx /= normR; ry /= normR; rz /= normR;
  }

  float roll = atan2f(2.0f * (rw*rx + ry*rz),
                      1.0f - 2.0f * (rx*rx + ry*ry));
  float pitchArg = 2.0f * (rw*ry - rz*rx);
  pitchArg = constrain(pitchArg, -1.0f, 1.0f);
  float pitch = asinf(pitchArg);
  float yaw = atan2f(2.0f * (rw*rz + rx*ry),
                     1.0f - 2.0f * (ry*ry + rz*rz));

  rollDeg = roll * RAD_TO_DEG;
  pitchDeg = pitch * RAD_TO_DEG;
  yawDeg = yaw * RAD_TO_DEG;
}

void finishMagCalibration() {
  magSpanX = magMaxX - magMinX;
  magSpanY = magMaxY - magMinY;
  magSpanZ = magMaxZ - magMinZ;
  magOffsetX = (magMaxX + magMinX) * 0.5f;
  magOffsetY = (magMaxY + magMinY) * 0.5f;
  magOffsetZ = (magMaxZ + magMinZ) * 0.5f;
  float rx = magSpanX * 0.5f;
  float ry = magSpanY * 0.5f;
  float rz = magSpanZ * 0.5f;
  float avg = (rx + ry + rz) / 3.0f;

  magCoverageValid = (magCalibrationSamples >= 100 &&
                      magSpanX >= 20.0f && magSpanY >= 20.0f && magSpanZ >= 20.0f &&
                      rx > 1.0f && ry > 1.0f && rz > 1.0f);
  if (magCoverageValid) {
    magScaleX = avg / rx; magScaleY = avg / ry; magScaleZ = avg / rz;
    magCalibrated = true;
  } else {
    magScaleX = magScaleY = magScaleZ = 1.0f;
    magCalibrated = false;
  }
  Serial.println("# MAG_CAL_RESULT: min/max approximation; validate coverage before use");
  Serial.print("# MagCalibrationSamples:"); Serial.println(magCalibrationSamples);
  Serial.print("# MagSpan_uT:"); Serial.print(magSpanX,3); Serial.print(','); Serial.print(magSpanY,3); Serial.print(','); Serial.println(magSpanZ,3);
  Serial.print("# MagOffset_uT:"); Serial.print(magOffsetX,3); Serial.print(','); Serial.print(magOffsetY,3); Serial.print(','); Serial.println(magOffsetZ,3);
  Serial.print("# MagScale:"); Serial.print(magScaleX,5); Serial.print(','); Serial.print(magScaleY,5); Serial.print(','); Serial.println(magScaleZ,5);
  Serial.print("# MagCoverageValid:"); Serial.println(magCoverageValid ? 1 : 0);
  Serial.print("# MagCalibrationValid:"); Serial.println(magCalibrated ? 1 : 0);
  if (!magCoverageValid) Serial.println("# WARNING: cobertura insuficiente; heading se marcara invalido. Repetir calibracion moviendo distintas caras/ejes.");
}

float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

void calculateMagHeading() {
  yawMagValid = false;
  yawMagDeg = -999.0f;
  yawMagRelativeDeg = -999.0f;
  // Caducidad de la muestra magnética para el cálculo de brújula compensada
  if (!magCalibrated || !magDataReady || (millis() - lastMagSampleMs > MAG_SAMPLE_TIMEOUT_MS)) return;

  // Remapeo de ejes y aplicación de matriz de montaje
  float mx = 0.0f, my = 0.0f, mz = 0.0f;
  getCalibratedMagBody(mx, my, mz);

  magNormCal = sqrtf(mx*mx + my*my + mz*mz);
  // Rechaza magnitudes manifiestamente improbables (campo terrestre típico: 25 a 65 uT)
  if (!isfinite(magNormCal) || magNormCal < 10.0f || magNormCal > 100.0f) return;

  float roll = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
  float pitchArg = 2.0f*(qw*qy - qz*qx); pitchArg = constrain(pitchArg, -1.0f, 1.0f);
  float pitch = asinf(pitchArg);

  float Xh = mx*cosf(pitch) + mz*sinf(pitch);
  float Yh = mx*sinf(roll)*sinf(pitch) + my*cosf(roll) - mz*sinf(roll)*cosf(pitch);
  if (!isfinite(Xh) || !isfinite(Yh) || (fabsf(Xh) < 0.001f && fabsf(Yh) < 0.001f)) return;

  yawMagDeg = atan2f(-Yh, Xh) * RAD_TO_DEG;
  if (!yawMagReferenceCaptured) { yawMagReferenceDeg = yawMagDeg; yawMagReferenceCaptured = true; }
  yawMagRelativeDeg = wrap180(yawMagDeg - yawMagReferenceDeg);
  yawMagValid = true;
}

// -------------------- Salida CSV ----------------------------
void printCSVHeader() {
  Serial.println(
    "Tiempo_ms,Roll_deg,Pitch_deg,Yaw_rel_deg,"
    "AccelX_g,AccelY_g,AccelZ_g,"
    "GyroX_dps,GyroY_dps,GyroZ_dps,"
    "MagRawX,MagRawY,MagRawZ,MagX_uT,MagY_uT,MagZ_uT,"
    "MagXCal_uT,MagYCal_uT,MagZCal_uT,MagNormRaw_uT,MagNormCal_uT,"
    "YawMag_deg,YawMag_rel_deg,YawMagValid,MagCoverageValid,"
    "Qw,Qx,Qy,Qz,I2C_Errors,MagErrors"
  );
}

void printCSVRow() {
  Serial.print(millis()); Serial.print(',');
  Serial.print(rollDeg, 3); Serial.print(',');
  Serial.print(pitchDeg, 3); Serial.print(',');
  Serial.print(yawDeg, 3); Serial.print(',');

  Serial.print(axG, 5); Serial.print(',');
  Serial.print(ayG, 5); Serial.print(',');
  Serial.print(azG, 5); Serial.print(',');

  Serial.print(gxDps, 4); Serial.print(',');
  Serial.print(gyDps, 4); Serial.print(',');
  Serial.print(gzDps, 4); Serial.print(',');

  Serial.print(magRawX); Serial.print(',');
  Serial.print(magRawY); Serial.print(',');
  Serial.print(magRawZ); Serial.print(',');
  Serial.print(magX_uT, 3); Serial.print(',');
  Serial.print(magY_uT, 3); Serial.print(',');
  Serial.print(magZ_uT, 3); Serial.print(',');
  Serial.print(magXCal, 3); Serial.print(',');
  Serial.print(magYCal, 3); Serial.print(',');
  Serial.print(magZCal, 3); Serial.print(',');
  Serial.print(magNormRaw, 3); Serial.print(',');
  Serial.print(magNormCal, 3); Serial.print(',');
  Serial.print(yawMagDeg, 3); Serial.print(',');
  Serial.print(yawMagRelativeDeg, 3); Serial.print(',');
  Serial.print(yawMagValid ? 1 : 0); Serial.print(',');
  Serial.print(magCoverageValid ? 1 : 0); Serial.print(',');

  Serial.print(qw, 6); Serial.print(',');
  Serial.print(qx, 6); Serial.print(',');
  Serial.print(qy, 6); Serial.print(',');
  Serial.print(qz, 6); Serial.print(',');
  Serial.print(i2cErrors); Serial.print(',');
  Serial.println(magErrors);
}

// -------------------- Arduino setup -------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // Inicializa el servicio GATT BLE inmediatamente para visibilidad temprana
  bleTelemetryBegin("MCL");
  bleTelemetrySetPhase(PHASE_BOOT);

  Serial.println();
  Serial.println("# MPU9250 v5.1 + BLE — Motion Controller Lab");
  Serial.println("# SDA=GPIO8, SCL=GPIO9");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!initMPU()) {
    Serial.println("# ERROR FATAL: no se pudo iniciar MPU.");
    bleTelemetrySetPhase(PHASE_FATAL);
    while (true) delay(1000);
  }

  if (!initMagnetometer()) {
    Serial.println("# ERROR FATAL: no se pudo iniciar AK8963.");
    Serial.println("# Revisar bypass, cableado, placa o sensor.");
    bleTelemetrySetPhase(PHASE_FATAL);
    while (true) delay(1000);
  }

  bleTelemetrySetPhase(PHASE_GYRO_CAL);
  if (!calibrateGyro()) {
    Serial.println("# ERROR FATAL: calibracion gyro fallida.");
    bleTelemetrySetPhase(PHASE_FATAL);
    while (true) delay(1000);
  }

  // Registra callback BLE de calibración e intenta cargar calibración de flash NVS
  bleSetCalibrationCallback(onBleCalibrationReceived);
  loadCalibrationFromNVS();

  if (magCalibrated && magCoverageValid) {
    Serial.println("# Calibracion de magnetometro disponible (NVS). Omitiendo cuenta atras de 30s.");
    bleTelemetrySetPhase(PHASE_SETTLING);
  } else {
    Serial.println("# CALIBRACION MAGNETOMETRO: mueve la placa libremente durante 30 segundos.");
    Serial.println("# (O conecta la app/gateway para inyectar calibracion guardada)...");
    uint32_t magCalStart = millis();
    uint32_t magReadLast = 0;
    while ((millis() - magCalStart < MAG_CAL_DURATION_MS) && !s_bleCalReceived) {
      uint32_t nowUs = micros();
      float dt = (nowUs - lastSampleUs) * 0.000001f;
      lastSampleUs = nowUs;
      if (readIMU() && dt > 0.0f && dt < 0.05f) madgwickUpdate6DOF(gxDps, gyDps, gzDps, axG, ayG, azG, dt);
      readMagnetometer();
      if (millis() - magReadLast >= 1000) {
        magReadLast = millis();
        uint8_t secsLeft = (MAG_CAL_DURATION_MS - (millis() - magCalStart)) / 1000;
        Serial.print("# MAG_CAL_SECONDS_LEFT:"); Serial.println(secsLeft);
        bleTelemetrySetPhase(PHASE_MAG_CAL, secsLeft);
      }
      delay(5);
    }
    if (!s_bleCalReceived) {
      finishMagCalibration();
      if (magCalibrated && magCoverageValid) {
        saveCalibrationToNVS();
      }
    }
  }

  Serial.println("# Estabilizando filtro IMU...");
  Serial.println("# No mover el sensor durante 1.5 segundos.");
  bleTelemetrySetPhase(PHASE_SETTLING);

  uint32_t settleStart = millis();
  lastSampleUs = micros();

  while (millis() - settleStart < 1500) {
    uint32_t nowUs = micros();
    float dt = (nowUs - lastSampleUs) * 0.000001f;
    lastSampleUs = nowUs;

    if (readIMU() && dt > 0.0f && dt < 0.05f) {
      readMagnetometer();
      if (magCalibrated && magCoverageValid && magDataReady) {
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        getCalibratedMagBody(mx, my, mz);
        madgwickUpdate9DOF(gxDps, gyDps, gzDps, axG, ayG, azG, mx, my, mz, dt);
      } else {
        madgwickUpdate6DOF(gxDps, gyDps, gzDps, axG, ayG, azG, dt);
      }
    }
    delay(5);
  }

  captureReference();
  calculateRelativeAngles();

  Serial.println("# Referencia angular capturada.");
  Serial.println("# Orientacion: Madgwick 9 ejes MARG (con correccion de ejes AK8963).");
  printCSVHeader();

  bleTelemetrySetPhase(PHASE_RUNNING);

  lastSampleUs = micros();
  lastCsvMs = millis();
  lastMagPrintMs = millis();
}

// -------------------- Arduino loop --------------------------
void loop() {
  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastSampleUs;
  if (elapsedUs < SAMPLE_PERIOD_US) return;

  // Detección de sobrecarga de ciclo (más del doble del periodo nominal)
  if (elapsedUs > 2 * SAMPLE_PERIOD_US) {
    bleTelemetryRecordLoopOverrun();
  }

  lastSampleUs = nowUs;
  float dt = elapsedUs * 0.000001f;
  if (dt <= 0.0f || dt > 0.05f) return;

  if (!readIMU()) return;

  // Lectura del magnetómetro (AK8963 a 100 Hz nominal en bucle de 200 Hz)
  bool newMag = readMagnetometer();

  // Gestión de caducidad y frescura (timeout de 50 ms compatible con periodo nominal de 10 ms)
  bool magFresh = (magDataReady && (millis() - lastMagSampleMs <= MAG_SAMPLE_TIMEOUT_MS));
  if (!magFresh && magDataReady) {
    magDataReady = false; // Caducó la muestra por falta de lecturas nuevas
  }

  // Fusión Madgwick 9DOF MARG si el magnetómetro tiene calibración válida y muestra vigente;
  // Fallback transparente a 6DOF en caso contrario (sin saltos ni discontinuidad de integración).
  if (magCalibrated && magCoverageValid && magFresh) {
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    getCalibratedMagBody(mx, my, mz);
    madgwickUpdate9DOF(gxDps, gyDps, gzDps, axG, ayG, azG, mx, my, mz, dt);
  } else {
    madgwickUpdate6DOF(gxDps, gyDps, gzDps, axG, ayG, azG, dt);
  }

  if (referenceCaptured) calculateRelativeAngles();

  calculateMagHeading();

  // Actualización de diagnósticos para BLE
  bleTelemetryUpdateDiagnostics(i2cErrors, magErrors);

  // Captura de instantánea en pila para telemetría BLE (sin String ni new)
  TelemetrySnapshot snapshot;
  snapshot.timestampMs = millis();
  snapshot.rollDeg = rollDeg;
  snapshot.pitchDeg = pitchDeg;
  snapshot.yawDeg = yawDeg;
  snapshot.yawMagRelativeDeg = yawMagRelativeDeg;
  snapshot.qw = qw;
  snapshot.qx = qx;
  snapshot.qy = qy;
  snapshot.qz = qz;
  snapshot.referenceCaptured = referenceCaptured;
  snapshot.magDataReady = magFresh;
  snapshot.magCoverageValid = magCoverageValid;
  snapshot.magCalibrated = magCalibrated;
  snapshot.yawMagValid = yawMagValid;

  bleTelemetryOnSample(snapshot);

  // Salida CSV existente por USB Serial
  uint32_t nowMs = millis();
  if (nowMs - lastCsvMs >= CSV_PERIOD_MS) {
    lastCsvMs = nowMs;
    printCSVRow();
  }
}
