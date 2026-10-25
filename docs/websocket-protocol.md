# Protocolo WebSocket v1 — Motion Controller Lab

El servidor WebSocket publica telemetría normalizada y estado del gateway en `ws://127.0.0.1:8765` (configurable).
Este protocolo desacopla completamente los visualizadores y motores de juego de la capa física BLE.

---

## 1. Conexión y Transporte

- **Dirección por defecto:** `ws://127.0.0.1:8765`.
- **Formato:** Mensajes de texto UTF-8 codificados en JSON estricto (`allow_nan=False`).
- **Política de entrega para telemetría:** *Latest-sample pattern* por cliente (cola `maxsize=1`).
  - Los clientes lentos nunca bloquean el gateway ni causan crecimiento de memoria; si un cliente no consume a tiempo, las muestras intermedias se descartan y se entrega siempre la muestra más reciente disponible.
- **Reconexión:** Los clientes pueden conectarse, desconectarse y reconectarse de forma independiente en cualquier momento.

---

## 2. Mensaje de Telemetría (`type: "telemetry"`)

Se publica a la tasa de llegada de las notificaciones BLE (objetivo nominal: 50 Hz).

### Esquema JSON

```json
{
  "type": "telemetry",
  "schema_version": 1,
  "device_id": "MCL-A1B2",
  "sequence": 1234,
  "sequence_raw": 1234,
  "lost_before": 0,
  "timestamp_ms": 45678,
  "host_time": 1761384000.123456,
  "extra_delay_ms": 1.25,
  "orientation": {
    "roll_deg": 1.23,
    "pitch_deg": -3.45,
    "yaw_rel_deg": 15.67,
    "yaw_mag_rel_deg": 14.81
  },
  "quaternion": {
    "w": 0.9902,
    "x": 0.0108,
    "y": -0.0301,
    "z": 0.1361
  },
  "validity": {
    "reference_captured": true,
    "euler_finite": true,
    "quat_valid": true,
    "mag_data_ready": true,
    "mag_coverage_valid": true,
    "mag_calibrated": true,
    "yaw_mag_valid": true
  },
  "warnings": []
}
```

### Definición de Campos

| Campo | Tipo | Descripción |
|---|---|---|
| `type` | string | Siempre `"telemetry"`. |
| `schema_version` | integer | Versión del esquema JSON (`1`). |
| `device_id` | string | Identificador del dispositivo (nombre BLE o MAC). |
| `sequence` | integer | Contador de secuencia monótono desenrollado (sin wrap en 65535). |
| `sequence_raw` | integer | Valor crudo de 16 bits recibido del sensor (`0..65535`). |
| `lost_before` | integer | Cantidad de paquetes faltantes detectados inmediatamente antes de este paquete. |
| `timestamp_ms` | integer | `millis()` monotónico capturado en el firmware al tomar la muestra. |
| `host_time` | float | Marca de tiempo monótona del host (`time.monotonic()`) al recibir la notificación. |
| `extra_delay_ms` | float | Retraso adicional relativo (jitter observable respecto al enlace más rápido). |
| `orientation` | object | Ángulos en grados, resolución 0.01°. |
| `orientation.roll_deg` | float \| null | Roll (inclinación lateral, eje X). `null` si no es finito. |
| `orientation.yaw_rel_deg` | float \| null | Yaw relativo de fusión Madgwick (9-DOF MARG con corrección magnética; fallback a 6-DOF). |
| `orientation.yaw_mag_rel_deg` | float \| null | Heading magnético relativo tilt-compensated. **`null` cuando es inválido, NUNCA `0`**. |
| `quaternion` | object \| null | Cuaternión absoluto normalizado `[w, x, y, z]` reconstruido del filtro. |
| `validity` | object | Indicadores booleanos de disponibilidad y validez. |
| `warnings` | array[string] | Advertencias del decodificador (ej. inconsistencias de banderas o fuera de rango). |

---

## 3. Mensaje de Estado (`type: "status"`)

Se publica periódicamente (1 Hz) o inmediatamente tras un cambio en la conexión.

### Esquema JSON

```json
{
  "type": "status",
  "schema_version": 1,
  "device_id": "MCL-A1B2",
  "gateway": {
    "connection": "connected",
    "mode": "ble",
    "ws_clients": 1
  },
  "stats": {
    "rate_hz": 49.8,
    "received": 2500,
    "lost": 2,
    "loss_ratio": 0.0008,
    "duplicates": 0,
    "reordered": 0,
    "device_resets": 0,
    "rejected": {},
    "warnings": {},
    "extra_delay_ms_mean": 1.4,
    "extra_delay_ms_max": 4.2,
    "device": {
      "phase": "running",
      "mag_cal_seconds_left": 0,
      "i2c_errors": 0,
      "mag_errors": 0,
      "loop_overruns": 0,
      "notify_drops": 0
    }
  },
  "timestamp": 1761384005.123
}
```

---

## 4. Reglas de Compatibilidad Futura

1. **Campos adicionales:** Los clientes deben ignorar cualquier clave desconocida que se agregue en el futuro.
2. **Incremento de versión:** Modificaciones incompatibles (cambios de nombres de campos, eliminación de campos o cambios semánticos de unidades) incrementarán `schema_version`.
3. **Manejo de Heading Inválido:** Si `yaw_mag_valid` es falso, los consumidores deben verificar `yaw_mag_rel_deg === null` y no asumir orientación magnética.
