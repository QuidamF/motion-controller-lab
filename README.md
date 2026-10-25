# Motion Controller Lab — 6-DOF / 9-DOF BLE Motion Controller

Sistema completo de control de movimiento y orientación inercial de ultra-baja latencia basado en **ESP32-C3 Super Mini** y sensor **MPU-9250 / AK8963**, con **Gateway Python desacoplado (BLE a WebSocket)**, calibración persistente en base de datos **SQLite**, inyección automática a memoria **Flash NVS** y herramientas gráficas de visualización y calibración 3D.

---

## 1. Esquemático de Conexión y Hardware

El firmware utiliza el bus I2C nativo del ESP32-C3 en modo *Fast Mode* (400 kHz).

```text
       ESP32-C3 Super Mini                         MPU-9250 / GY-91
    +-----------------------+                    +------------------+
    |                       |                    |                  |
    |                  3.3V |------------------->| VCC (3.3V)       |
    |                   GND |------------------->| GND              |
    |                       |                    |                  |
    |        GPIO 9 (SCL)   |------------------->| SCL              |
    |        GPIO 8 (SDA)   |<------------------>| SDA              |
    |                       |                    |                  |
    |                   GND |------------------->| AD0 (I2C = 0x68) |
    |                       |                    | NCS (I2C Mode)   |--- (Flotante o 3.3V)
    +-----------------------+                    +------------------+
```

### Tabla de Conexionado

| Pin Sensor MPU-9250 | Pin ESP32-C3 Super Mini | Función | Descripción |
| :--- | :--- | :--- | :--- |
| **VCC** | **3.3V** | Alimentación | 3.3V lógico (o 5V si tu módulo tiene regulador integrado). |
| **GND** | **GND** | Tierra común | Masa compartida. |
| **SCL** | **GPIO 9** | I2C Clock | Reloj del bus I2C (400 kHz). |
| **SDA** | **GPIO 8** | I2C Data | Datos bidireccionales de telemetría. |
| **AD0** | **GND** | Dirección I2C | Fija la dirección del MPU en `0x68`. |
| **NCS** | *Flotante o 3.3V* | Modo Bus | Mantener en alto para bus I2C. |

> Guía de hardware detallada: [hardware_schematic.md](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/docs/hardware_schematic.md).

---

## 2. Carga del Firmware en el ESP32-C3

El código fuente del firmware se encuentra en [firmware/motion_controller_ble](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/firmware/motion_controller_ble).

### Opción A: Vía Arduino CLI (Recomendado)
```bash
# Compilar y subir por USB Serial/JTAG (reemplaza COM7 por tu puerto)
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc firmware/motion_controller_ble
arduino-cli upload -p COM7 --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc firmware/motion_controller_ble
```

### Opción B: Vía Arduino IDE
1. Placa: **ESP32C3 Dev Module** (o ESP32-C3 Super Mini).
2. Opciones de configuración:
   - **USB CDC On Boot:** `Enabled`
   - **Flash Frequency:** `80MHz`
   - **Upload Speed:** `921600`
3. Abre [motion_controller_ble.ino](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/firmware/motion_controller_ble/motion_controller_ble.ino) y pulsa **Subir**.

---

## 3. Instalación y Ejecución del Gateway Python

El gateway actúa como puente de alto rendimiento: recibe paquetes BLE GATT de 20 bytes a 50 Hz, gestiona la calibración SQLite y expone un servidor WebSocket local en `ws://127.0.0.1:8765`.

### Instalación del Entorno Virtual
```bash
cd motion-controller-lab/gateway

# 1. Crear entorno virtual
python -m venv .venv

# 2. Instalar el paquete en modo editable
.\.venv\Scripts\pip install -e .
```

### Ejecutar el Gateway

**Con Hardware Real (BLE):**
```bash
.\.venv\Scripts\python -m motion_gateway.main --name MCL-
```
*(El gateway escaneará automáticamente periféricos BLE que comiencen por `MCL-` y se conectará en milisegundos).*

**Modo Simulado (Sin Hardware):**
```bash
.\.venv\Scripts\python -m motion_gateway.main --sim
```

---

## 4. Uso de las Interfaces Gráficas Web

Las herramientas se abren directamente en cualquier navegador moderno sin requerir servidores HTTP adicionales.

### A. Visualizador 3D en Tiempo Real ([tools/visualizer3d.html](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/tools/visualizer3d.html))
- **Visualización 3D WebGL a 60+ FPS:** Renderizado fluido con Three.js e interpolación esférica (Slerp).
- **Modelos Intercambiables:**
  - Placa **ESP32-C3 Super Mini + MPU-9250**.
  - **Dron Cuadricóptero**.
  - **Mando VR / Controller**.
  - **Espada / Raqueta de Tenis**.
  - **Bote de Grafiti Virtual:** Presiona `R` o mantén el botón del gatillo para pulverizar pintura aerosol sobre el muro 3D.
- **Estimación de Movimiento 6-DOF:** Cálculo de velocidad y desplazamiento tridimensional (ideal para dinámicas deportivas y gestos).
- **Orientación de Montaje IMU 3D:**
  - Presets rápidos: `0° Estándar`, `+90° Yaw` (protoboard), `-90° Yaw`, `180° Yaw`, `+90° Roll` (de lado), `+90° Pitch` (vertical).
  - Entradas numéricas directas para **Yaw (Z)**, **Pitch (Y)** y **Roll (X)**.
  - **🎯 Detectar con 2 Gestos (Drone):** Calcula automáticamente la matriz 3D completa:
    1. *Paso 1 (2s):* Reposo plano en mesa (determina vector gravedad $\vec{u}_z$ -> Pitch y Roll).
    2. *Paso 2 (2s):* Inclinación hacia adelante (determina eje transversal $\vec{u}_y$ -> Yaw y valida ortonormalidad).
- **Atajos de Teclado:**
  - <kbd>Espacio</kbd>: **Tare / Centrar** (fija la postura actual como origen cero).
  - <kbd>R</kbd>: Rociar spray de pintura.
  - <kbd>T</kbd>: Activar/desactivar estela de movimiento.
  - <kbd>Click Izquierdo</kbd>: Orbitar cámara.
  - <kbd>Scroll</kbd>: Zoom in / out.

### B. Asistente de Calibración SQLite ([tools/calibration.html](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/tools/calibration.html))
- **Mapeo Esférico 3D:** Muestra en tiempo real la esfera magnética eliminando interferencias de *Hard-Iron* (desplazamiento del centro) y *Soft-Iron* (deformación elipsoidal).
- **Base de Datos SQLite (`calibration.db`):** Cada periferico se calibra **una sola vez** y se guarda permanentemente.
- **Inyección por BLE y NVS:** Al conectar el dispositivo, el Gateway inyecta la calibración y el montaje 3D por BLE. El ESP32 los almacena en su memoria Flash NVS y reduce el tiempo de arranque de ~35 segundos a solo **~3 segundos**, saltándose la cuenta atrás inicial.

---

## 5. Conexión al Gateway desde Cualquier Plataforma

Cualquier juego, motor o aplicación externa puede consumir la telemetría conectándose como cliente WebSocket a:

```
ws://127.0.0.1:8765
```

### Esquema del Paquete JSON (50 Hz)

```json
{
  "type": "telemetry",
  "schema_version": 1,
  "device_id": "MCL-0001",
  "sequence": 1420,
  "timestamp_ms": 134200,
  "orientation": {
    "roll_deg": -1.25,
    "pitch_deg": 4.10,
    "yaw_rel_deg": 89.40,
    "yaw_mag_rel_deg": 90.15
  },
  "quaternion": {
    "w": 0.7071,
    "x": 0.0012,
    "y": 0.0045,
    "z": 0.7070
  },
  "validity": {
    "euler_finite": true,
    "quat_valid": true,
    "yaw_mag_valid": true
  }
}
```

---

### Ejemplos de Integración Listos para Usar

El repositorio incluye plantillas completas y probadas en el directorio [examples/](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/examples):

#### A. Python ([examples/python_client.py](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/examples/python_client.py))
```python
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://127.0.0.1:8765") as ws:
        async for msg in ws:
            data = json.loads(msg)
            if data.get("type") == "telemetry":
                ori = data["orientation"]
                q = data["quaternion"]
                print(f"Roll: {ori['roll_deg']}° | Pitch: {ori['pitch_deg']}° | Yaw: {ori['yaw_rel_deg']}°")

asyncio.run(main())
```

#### B. Godot 4 — GDScript ([examples/godot_example.gd](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/examples/godot_example.gd))
Adjunta el script a cualquier `Node3D`:
```gdscript
extends Node3D

var _socket = WebSocketPeer.new()

func _ready():
    _socket.connect_to_url("ws://127.0.0.1:8765")

func _process(delta):
    _socket.poll()
    while _socket.get_available_packet_count() > 0:
        var msg = JSON.parse_string(_socket.get_packet().get_string_from_utf8())
        if msg and msg.get("type") == "telemetry" and msg.get("quaternion"):
            var q = msg["quaternion"]
            # Mapeo a convención Godot (Y arriba, -Z adelante, X derecha):
            quaternion = Quaternion(q.x, -q.z, -q.y, q.w)
```

#### C. Unity 3D — C# ([examples/UnityMotionController.cs](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/examples/UnityMotionController.cs))
Convierte coordenadas de IMU al sistema zurdo de Unity:
```csharp
// Dentro de OnMessage en Unity:
float unityX = msg.quaternion.x;
float unityY = -msg.quaternion.z; // Invertir Yaw según mano
float unityZ = -msg.quaternion.y;
float unityW = msg.quaternion.w;

transform.rotation = new Quaternion(unityX, unityY, unityZ, unityW);
```

#### D. Web / HTML5 Minimal ([examples/web_client_minimal.html](file:///c:/Users/edgar/Documents/Proyectos2026/IMU/motion-controller-lab/examples/web_client_minimal.html))
```javascript
const ws = new WebSocket("ws://127.0.0.1:8765");
ws.onmessage = (e) => {
  const data = JSON.parse(e.data);
  if (data.type === "telemetry") {
    console.log("Orientación:", data.orientation);
  }
};
```

---

## 6. Pruebas Automatizadas

El proyecto cuenta con una suite integral de 52 pruebas unitarias y de integración:

```bash
cd gateway
.\.venv\Scripts\python -m unittest discover -s tests -v
```

```
Ran 52 tests in 1.28s
OK
```

---

## 7. Estructura del Repositorio

```
IMU/
├── MPU9250_Magnetometer_v5_1_Validated_Heading/  <-- Sketch original intacto
└── motion-controller-lab/
    ├── docs/
    │   ├── hardware_schematic.md <-- Diagrama y tabla de conexión de pines
    │   ├── architecture.md       <-- Arquitectura del sistema y flujo de datos
    │   ├── ble-protocol.md       <-- Protocolo binario BLE GATT (20 bytes)
    │   └── websocket-protocol.md <-- Especificación del esquema JSON
    ├── examples/
    │   ├── python_client.py      <-- Cliente CLI en Python
    │   ├── godot_example.gd      <-- Script de integración para Godot 4
    │   ├── UnityMotionController.cs <-- Script MonoBehaviour para Unity C#
    │   └── web_client_minimal.html  <-- Cliente web HTML5 mínimo
    ├── firmware/
    │   └── motion_controller_ble/ <-- Firmware ESP32-C3 + MPU-9250 + AK8963
    ├── gateway/
    │   ├── pyproject.toml        <-- Paquete pip motion-gateway
    │   ├── src/motion_gateway/
    │   │   ├── protocol.py       <-- Serializador y decodificador binario
    │   │   ├── telemetry.py      <-- Normalizador y métricas a 50 Hz
    │   │   ├── ble_client.py     <-- Cliente BLE GATT (Bleak)
    │   │   ├── calibration_db.py <-- Base de datos SQLite persistente
    │   │   └── websocket_server.py <-- Servidor WebSocket local
    │   └── tests/                <-- 52 tests unitarios automatizados
    └── tools/
        ├── visualizer3d.html     <-- Visualizador 3D WebGL con modelos, PFD y grafiti
        ├── calibration.html      <-- Asistente gráfico de calibración 3D SQLite
        └── diagnostic.html       <-- Dashboard numérico de diagnóstico en tiempo real
```
