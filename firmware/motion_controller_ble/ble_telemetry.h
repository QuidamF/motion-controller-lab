#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include <Arduino.h>

// UUIDs del protocolo Motion Controller Lab v1
#define SERVICE_UUID          "4d430001-8f3a-4c2b-9e71-2a5c6d7e8f90"
#define TELEMETRY_CHAR_UUID   "4d430002-8f3a-4c2b-9e71-2a5c6d7e8f90"
#define STATUS_CHAR_UUID      "4d430003-8f3a-4c2b-9e71-2a5c6d7e8f90"
#define CALIBRATION_CHAR_UUID "4d430004-8f3a-4c2b-9e71-2a5c6d7e8f90"

// Banderas de protocolo (bits 0..7)
#define FLAG_REF_CAPTURED   (1 << 0)
#define FLAG_EULER_FINITE   (1 << 1)
#define FLAG_QUAT_VALID     (1 << 2)
#define FLAG_MAG_DATA_READY (1 << 3)
#define FLAG_MAG_COV_VALID  (1 << 4)
#define FLAG_MAG_CALIBRATED (1 << 5)
#define FLAG_YAW_MAG_VALID  (1 << 6)

enum FirmwarePhase : uint8_t {
  PHASE_BOOT = 0,
  PHASE_GYRO_CAL = 1,
  PHASE_MAG_CAL = 2,
  PHASE_SETTLING = 3,
  PHASE_RUNNING = 4,
  PHASE_FATAL = 255
};

struct TelemetrySnapshot {
  uint32_t timestampMs;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float yawMagRelativeDeg;
  float qw, qx, qy, qz;
  bool referenceCaptured;
  bool magDataReady;
  bool magCoverageValid;
  bool magCalibrated;
  bool yawMagValid;
};

// Paquete binario de calibración (32 bytes)
struct __attribute__((packed)) BleCalibrationPacket {
  uint8_t version;          // 1
  uint8_t flags;            // bit 0: mag_valid, bit 1: mount_valid, bit 2: save_nvs
  int16_t mountYawDeg10;    // deg * 10 (-1800..1800)
  int16_t mountPitchDeg10;  // deg * 10 (-900..900)
  int16_t mountRollDeg10;   // deg * 10 (-1800..1800)
  float magOffsetX;         // uT
  float magOffsetY;         // uT
  float magOffsetZ;         // uT
  float magScaleX;
  float magScaleY;
  float magScaleZ;
};

typedef void (*CalibrationReceivedCallback)(const BleCalibrationPacket& packet);

// Registra la función callback invocada cuando se escribe la característica de calibración
void bleSetCalibrationCallback(CalibrationReceivedCallback cb);

// Actualiza el valor leído de la característica de calibración
void bleTelemetrySetCalibrationData(const BleCalibrationPacket& packet);

// Inicializa el servicio GATT y comienza el advertising
void bleTelemetryBegin(const char* baseName = "MCL");

// Actualiza la fase de operación y segundos restantes de calibración
void bleTelemetrySetPhase(FirmwarePhase phase, uint8_t secondsLeft = 0);

// Invocado en cada muestra del ciclo IMU (200 Hz).
// Transmite por BLE cada 4 muestras (nominal 50 Hz) priorizando siempre la más reciente.
void bleTelemetryOnSample(const TelemetrySnapshot& sample);

// Registra una sobrecarga de ciclo (cuando elapsedUs > 2 * SAMPLE_PERIOD_US)
void bleTelemetryRecordLoopOverrun();

// Actualiza contadores de errores para el paquete de estado
void bleTelemetryUpdateDiagnostics(uint32_t i2cErrors, uint32_t magErrors);

// Indica si hay un cliente central BLE conectado
bool bleIsConnected();

#endif // BLE_TELEMETRY_H

