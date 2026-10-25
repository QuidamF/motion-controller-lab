#include "ble_telemetry.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

static BLEServer* pServer = nullptr;
static BLECharacteristic* pTelemChar = nullptr;
static BLECharacteristic* pStatusChar = nullptr;
static BLECharacteristic* pCalibChar = nullptr;

static CalibrationReceivedCallback s_calibCallback = nullptr;
static volatile bool s_deviceConnected = false;
static uint16_t s_sequence = 0;
static uint32_t s_sampleCounter = 0;

static volatile FirmwarePhase s_currentPhase = PHASE_BOOT;
static volatile uint8_t s_magCalSecsLeft = 0;
static volatile uint32_t s_i2cErrors = 0;
static volatile uint32_t s_magErrors = 0;
static volatile uint32_t s_loopOverruns = 0;
static volatile uint16_t s_notifyDrops = 0;
static uint32_t s_lastStatusSendMs = 0;

static const int16_t INVALID_I16 = -32768; // 0x8000

class BleServerCallbacksHandler : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    s_deviceConnected = true;
  }

  void onDisconnect(BLEServer* server) override {
    s_deviceConnected = false;
    // Permite reconexión inmediata sin reiniciar el sensor
    BLEDevice::startAdvertising();
  }
};

class TelemCharCallbacksHandler : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) override {
    // Registra descartes de notificaciones por cola GATT llena, falta de memoria o notificaciones deshabilitadas
    if (s == Status::ERROR_GATT || s == Status::ERROR_NOTIFY_DISABLED || s == Status::ERROR_NO_CLIENT) {
      s_notifyDrops++;
    }
  }
};

class CalibCharCallbacksHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String val = pCharacteristic->getValue();
    if (val.length() >= sizeof(BleCalibrationPacket)) {
      BleCalibrationPacket packet;
      memcpy(&packet, val.c_str(), sizeof(packet));
      if (s_calibCallback != nullptr) {
        s_calibCallback(packet);
      }
    }
  }
};


static int16_t encodeAngle(float angleDeg, float limit) {
  if (!isfinite(angleDeg) || fabsf(angleDeg) > limit + 0.001f) {
    return INVALID_I16;
  }
  int32_t raw = (int32_t)roundf(angleDeg * 100.0f);
  if (raw < -32767) raw = -32767;
  if (raw > 32767) raw = 32767;
  return (int16_t)raw;
}

static uint32_t packQuaternion(float w, float x, float y, float z, bool& valid) {
  float comps[4] = {w, x, y, z};
  for (int i = 0; i < 4; i++) {
    if (!isfinite(comps[i])) {
      valid = false;
      return 0;
    }
  }
  float normSq = w * w + x * x + y * y + z * z;
  if (normSq < 0.90f || normSq > 1.10f) {
    valid = false;
    return 0;
  }
  float invNorm = 1.0f / sqrtf(normSq);
  for (int i = 0; i < 4; i++) comps[i] *= invNorm;

  int maxIdx = 0;
  float maxVal = fabsf(comps[0]);
  for (int i = 1; i < 4; i++) {
    float a = fabsf(comps[i]);
    if (a > maxVal) {
      maxVal = a;
      maxIdx = i;
    }
  }

  // Garantiza que la componente omitida sea positiva (q y -q representan la misma rotación)
  if (comps[maxIdx] < 0.0f) {
    for (int i = 0; i < 4; i++) comps[i] = -comps[i];
  }

  uint32_t packed = ((uint32_t)maxIdx) << 30;
  int shift = 20;
  // Factor de escala: 511 / (1 / sqrt(2)) = ~722.6632f
  for (int i = 0; i < 4; i++) {
    if (i == maxIdx) continue;
    int32_t v = (int32_t)roundf(comps[i] * 722.6632f);
    if (v < -511) v = -511;
    if (v > 511) v = 511;
    packed |= ((uint32_t)(v & 0x3FF)) << shift;
    shift -= 10;
  }
  valid = true;
  return packed;
}

static void sendStatusNotification() {
  if (!s_deviceConnected || pStatusChar == nullptr) return;

  uint8_t buffer[20];
  memset(buffer, 0, sizeof(buffer));

  buffer[0] = 1; // version = 1
  buffer[1] = (uint8_t)s_currentPhase;
  buffer[2] = s_magCalSecsLeft;
  buffer[3] = 0;

  uint32_t i2c = s_i2cErrors;
  uint32_t mag = s_magErrors;
  uint32_t overruns = s_loopOverruns;
  uint16_t drops = s_notifyDrops;

  memcpy(&buffer[4], &i2c, 4);
  memcpy(&buffer[8], &mag, 4);
  memcpy(&buffer[12], &overruns, 4);
  memcpy(&buffer[16], &drops, 2);

  pStatusChar->setValue(buffer, sizeof(buffer));
  pStatusChar->notify();
}

void bleTelemetryBegin(const char* baseName) {
  // Genera nombre único estable a partir de los 2 últimos bytes de la MAC física (eFuse)
  // evitando invocar BLEDevice::getAddress() antes de BLEDevice::init() (D-01)
  uint64_t mac = ESP.getEfuseMac();
  uint8_t b4 = (uint8_t)(mac >> 32);
  uint8_t b5 = (uint8_t)(mac >> 40);
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "-%02X%02X", b4, b5);
  String devName = String(baseName) + suffix;

  BLEDevice::init(devName.c_str());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacksHandler());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTelemChar = pService->createCharacteristic(
    TELEMETRY_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelemChar->setCallbacks(new TelemCharCallbacksHandler());
  pTelemChar->addDescriptor(new BLE2902());

  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());

  pCalibChar = pService->createCharacteristic(
    CALIBRATION_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pCalibChar->setCallbacks(new CalibCharCallbacksHandler());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Funciones de ayuda para iPhone / Windows
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("# BLE GATT activo con nombre: ");
  Serial.println(devName);
}

void bleSetCalibrationCallback(CalibrationReceivedCallback cb) {
  s_calibCallback = cb;
}

void bleTelemetrySetCalibrationData(const BleCalibrationPacket& packet) {
  if (pCalibChar != nullptr) {
    pCalibChar->setValue((uint8_t*)&packet, sizeof(packet));
  }
}


void bleTelemetrySetPhase(FirmwarePhase phase, uint8_t secondsLeft) {
  s_currentPhase = phase;
  s_magCalSecsLeft = secondsLeft;
  sendStatusNotification();
}

void bleTelemetryRecordLoopOverrun() {
  s_loopOverruns++;
}

void bleTelemetryUpdateDiagnostics(uint32_t i2cErrors, uint32_t magErrors) {
  s_i2cErrors = i2cErrors;
  s_magErrors = magErrors;
}

bool bleIsConnected() {
  return s_deviceConnected;
}

void bleTelemetryOnSample(const TelemetrySnapshot& sample) {
  // Envía 1 paquete cada 4 ciclos IMU (200 Hz / 4 = 50 Hz)
  s_sampleCounter++;
  if ((s_sampleCounter % 4) != 0) {
    return;
  }

  // Si no hay suscriptor BLE conectado, no procesa ni retiene paquetes
  if (!s_deviceConnected || pTelemChar == nullptr) {
    return;
  }

  // 1 Hz Status heartbeat
  uint32_t nowMs = millis();
  if (nowMs - s_lastStatusSendMs >= 1000) {
    s_lastStatusSendMs = nowMs;
    sendStatusNotification();
  }

  // Empaquetado binario de 20 bytes en pila (sin asignación dinámica ni String)
  uint8_t buffer[20];

  int16_t encRoll = encodeAngle(sample.rollDeg, 180.0f);
  int16_t encPitch = encodeAngle(sample.pitchDeg, 90.0f);
  int16_t encYaw = encodeAngle(sample.yawDeg, 180.0f);

  int16_t encMag = INVALID_I16;
  if (sample.yawMagValid && sample.yawMagRelativeDeg > -900.0f) {
    encMag = encodeAngle(sample.yawMagRelativeDeg, 180.0f);
  }

  bool quatValid = false;
  uint32_t packedQuat = packQuaternion(sample.qw, sample.qx, sample.qy, sample.qz, quatValid);

  uint8_t flags = 0;
  if (sample.referenceCaptured) flags |= FLAG_REF_CAPTURED;
  if (sample.magDataReady)      flags |= FLAG_MAG_DATA_READY;
  if (sample.magCoverageValid)  flags |= FLAG_MAG_COV_VALID;
  if (sample.magCalibrated)     flags |= FLAG_MAG_CALIBRATED;

  if (encRoll != INVALID_I16 && encPitch != INVALID_I16 && encYaw != INVALID_I16) {
    flags |= FLAG_EULER_FINITE;
  } else {
    encRoll = encPitch = encYaw = INVALID_I16;
  }

  if (quatValid) {
    flags |= FLAG_QUAT_VALID;
  } else {
    packedQuat = 0;
  }

  if (encMag != INVALID_I16 && sample.yawMagValid) {
    flags |= FLAG_YAW_MAG_VALID;
  } else {
    encMag = INVALID_I16;
  }

  uint16_t seq = s_sequence++;
  uint32_t ts = sample.timestampMs;

  buffer[0] = 1; // protocol_version = 1
  buffer[1] = flags;
  memcpy(&buffer[2], &seq, 2);
  memcpy(&buffer[4], &ts, 4);
  memcpy(&buffer[8], &encRoll, 2);
  memcpy(&buffer[10], &encPitch, 2);
  memcpy(&buffer[12], &encYaw, 2);
  memcpy(&buffer[14], &encMag, 2);
  memcpy(&buffer[16], &packedQuat, 4);

  pTelemChar->setValue(buffer, sizeof(buffer));
  pTelemChar->notify();
}
