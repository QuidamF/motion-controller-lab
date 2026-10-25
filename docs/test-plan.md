# Plan de Pruebas y Matriz de Validación — Motion Controller Lab

Este documento consolida las evidencias técnicas del sistema clasificadas en cinco niveles independientes de verificación: código estático, pruebas automatizadas, simulación sintética, validación física en hardware y limitaciones abiertas.

---

## 1. Resultados Comprobados Mediante Código

Auditoría directa sobre el código fuente de firmware C++ y componentes del gateway:

- **Estructura y alineación de paquetes (20 bytes):**
  - Little-endian estricto tanto en ESP32-C3 (RISC-V) como en Python (`struct.unpack('<BBHIhhhhI')`).
  - Tamaño invariable de 20 bytes en telemetría y 20 bytes en estado (compatible con MTU por defecto ATT de 23 bytes).
- **Algoritmo de orientación Madgwick 9-DOF MARG:**
  - Inclusión completa del gradiente magnético hacia el vector de referencia de la Tierra $\mathbf{b} = [b_x, 0, b_z]$.
  - Comprobación de que `_2bx` y `_2bz` en [`motion_controller_ble.ino`](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/firmware/motion_controller_ble/motion_controller_ble.ino) coinciden formalmente con la formulación canónica de Madgwick (2010).
  - Remapeo de fábrica de los ejes del magnetómetro AK8963 a la terna del MPU-9250: $X_{body} = +Y_{mag}, Y_{body} = +X_{mag}, Z_{body} = -Z_{mag}$.
  - Corrección de unidades en el fallback a 6-DOF (`madgwickUpdate6DOFRad`), eliminando la atenuación espuria de $57.3\times$.
  - Watchdog de frescura de 50 ms para el AK8963 (100 Hz), con fallback limpio a 6-DOF ante congelamiento o saturación (`HOFL`).
- **Métricas e instrumentación de descartes BLE:**
  - Implementación de callback `TelemCharCallbacksHandler` que captura fallos en la cola del radio (`ERROR_GATT`, `ERROR_NOTIFY_DISABLED`) e incrementa `s_notifyDrops`.
- **Inicialización BLE segura:**
  - Lectura de hardware eFuse (`ESP.getEfuseMac()`) antes de `BLEDevice::init()` para generar un nombre dinámico determinista `MCL-XXXX` (ej. `MCL-43EA`) sin arrojar errores de stack no inicializado.

---

## 2. Resultados Comprobados Mediante Pruebas Automatizadas

La suite automatizada se ejecuta localmente en el entorno virtual de Python:

```bash
cd gateway
.\.venv\Scripts\python -m unittest discover -s tests -v
```

### Resumen de Ejecución (45 Pruebas Totales — 100% PASS)

| Módulo de Pruebas | Casos | Estado | Cobertura Demostrada |
|---|---|---|---|
| [`test_protocol.py`](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/gateway/tests/test_protocol.py) | 21 | PASS | Vector dorado byte a byte, límites angulares `[-180°, 180°]` y `[-90°, 90°]`, descarte de NaN/Inf, valores centinela (`-999` / `0x8000`), heading inválido como `None` (nunca `0`), empaquetado de cuaternión *Smallest-Three* (error angular peor caso $0.2199^\circ < 0.28^\circ$, media $0.0845^\circ$ sobre 50,000 orientaciones), inmunidad ante 5,000 paquetes truncados/corruptos aleatorios. |
| [`test_telemetry.py`](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/gateway/tests/test_telemetry.py) | 16 | PASS | Desenrollado de secuencia con wrap en 65535, detección de pérdidas (`lost_before`), descarte de paquetes duplicados y reordenados, detección de reinicio de hardware (salto > 1000 hacia atrás), cálculo de tasa en ventana deslizante (50.0 Hz), jitter observable, normalización a JSON schema v1, discriminación semántica entre descartes del emisor (`device.notify_drops`) y pérdidas por radio (`seq.lost`). |
| [`test_gateway_simulation.py`](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/gateway/tests/test_gateway_simulation.py) | 5 | PASS | Cadencia del simulador a 50 y 100 Hz con inyección de anomalías, aislamiento de cliente lento con política *latest-sample* (`ClientSlot` con `maxsize=1` sin crecimiento de memoria), streaming concurrente multicliente a 50 Hz, y desconexión/reconexión limpia de clientes WebSocket. |
| [`test_orientation_algorithm.py`](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/gateway/tests/test_orientation_algorithm.py) | 3 | PASS | Regresión D-03 (integración nominal de 90° en 6-DOF sin doble conversión deg→rad), regresión D-04 (equivalencia matemática estocástica contra `MadgwickAHRS.c`), regresión D-05 (watchdog de 50 ms ante corte magnético y recuperación limpia a 9-DOF). |

---

## 3. Resultados Comprobados Mediante Simulación

Verificación del pipeline de datos sin requerir hardware físico:

1. **Simulación Nominal (`--sim`):**
   - Publicación a 50.0 Hz sostenidos con generación sintética de orientaciones en rotación continua.
   - Transmisión continua a WebSocket local (`ws://127.0.0.1:8765`).
2. **Inyección de Fallos Controlada (`--sim-drops 0.05 --sim-disconnects 15`):**
   - El normalizador detecta y cuantifica exactamente la tasa de paquetes perdidos inyectada.
   - Las desconexiones forzadas activan el backoff exponencial y restablecen la continuidad del flujo sin interrupciones del proceso.
3. **Validación en Cliente Web (`diagnostic.html`):**
   - Visualización fluida de los instrumentos de actitud (Roll, Pitch, Yaw inercial y brújula).
   - Ausencia de saltos de 360° en la aguja al cruzar la discontinuidad $\pm 180^\circ$ gracias al desenrollado de fase continuo.

---

## 4. Resultados Comprobados Físicamente (Sesiones en Hardware Real)

Evidencia experimental capturada con microcontrolador físico y sensor:

### Entorno Físico
- **Microcontrolador:** ESP32-C3 Super Mini (RISC-V).
- **Sensor:** MPU-9250 (Acelerómetro + Giróscopo) + AK8963 (Magnetómetro) vía I2C.
- **Plataforma Host:** Windows 11 / Python 3.14 con WinRT Bluetooth y Bleak v3.0.2.
- **Dirección MAC Dispositivo:** `70:AF:09:15:43:EA`.

### Métricas Observadas en Sesión de Streaming
1. **Streaming Sostenido:**
   - **Muestras recibidas:** **87,387 paquetes** a **49.5 Hz**.
   - **Pérdida en el aire:** Únicamente **4 paquetes** perdidos en más de 87,000 muestras (**$0.005\%$ de pérdida** / 99.995% de entrega).
   - **Jitter observable:** Retraso relativo adicional medio de 35 a 42 ms (asociado a la periodicidad de los intervalos de conexión BLE de Windows).
2. **Descubrimiento y Reconexión:**
   - Descubrimiento automático por `SERVICE_UUID` y prefijo `MCL-`.
   - Tras desconexión y reinicio, el gateway reconecta de forma automática y transparente sin intervención manual.
3. **Concurrencia USB Serial y BLE:**
   - Salida simultánea de telemetría CSV por puerto serie CDC a 115200 baudios y notificaciones BLE a 50 Hz sin degradación de la tasa de integración IMU a 200 Hz.

---

## 5. Limitaciones Todavía Abiertas

1. **Protocolo Físico de Precisión Angular en Banco:**
   - Las pruebas de rotación ortogonal exacta ($90.0^\circ$ y $180.0^\circ$ con escuadra y transportador) y el análisis de deriva estática prolongada quedan catalogados como protocolo físico reproducible pendiente de ejecución formal por parte del operador con instrumental de banco.
2. **Entornos con Distorsión Magnética Severa:**
   - La calibración actual del magnetómetro se basa en una aproximación de envolvente mín/máx de esfera durante los 30 segundos de arranque. En entornos con presencia de masas ferromagnéticas variables (hierro blando no uniforme), se requiere verificar que `magCoverageValid` sea 1 antes de confiar en la referencia de rumbo absoluto.
3. **Medición de Latencia Unidireccional Absoluta:**
   - Sin sincronización por hardware PTP/NTP entre el oscilador del ESP32 y el reloj del host, la latencia de extremo a extremo absoluta no es directamente observable; solo se cuantifica el jitter relativo y la tasa de entrega.
