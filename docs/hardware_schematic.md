# Esquemático de Conexión y Hardware — ESP32-C3 Super Mini + MPU-9250

Este documento detalla el conexionado físico entre la placa de desarrollo **ESP32-C3 Super Mini** y el sensor inercial de 9 grados de libertad **MPU-9250** (acelerómetro + giróscopo MPU-6500 + magnetómetro AK8963).

---

## 1. Diagrama de Conexión (I2C)

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

---

## 2. Tabla de Pines

| Pin Sensor MPU-9250 | Pin ESP32-C3 Super Mini | Función | Notas |
| :--- | :--- | :--- | :--- |
| **VCC** | **3.3V** | Alimentación lógica y analógica | Los módulos GY-91/9250 suelen integrar regulador LDO a 3.3V. |
| **GND** | **GND** | Referencia de tierra común | Masa compartida. |
| **SCL** | **GPIO 9** | Reloj I2C (*I2C Clock*) | Frecuencia estándar del bus: 400 kHz (*Fast Mode*). |
| **SDA** | **GPIO 8** | Datos I2C (*I2C Data*) | Línea bidireccional de telemetría y configuración. |
| **AD0** | **GND** | Selección de dirección I2C | Conectar a GND fija la dirección del MPU en `0x68`. |
| **NCS** | *Flotante o 3.3V* | Chip Select SPI | En nivel alto o flotante habilita comunicación por I2C. |
| **INT** | *No Conectado* | Interrupción | Opcional (el firmware muestrea el sensor por polling a 200 Hz). |
| **ECL / EDA** | *No Conectado* | I2C Auxiliar | No se usan (el magnetómetro AK8963 se lee en modo *I2C Bypass* en `0x0C`). |

> [!NOTE]
> La mayoría de los módulos comerciales **MPU-9250** o **GY-91** ya integran en la placa dos resistencias *pull-up* de 4.7 kΩ o 10 kΩ en las líneas SDA y SCL hacia 3.3V, por lo que no es necesario añadir resistencias externas en la protoboard.

---

## 3. Direcciones I2C en el Bus

El firmware inicializa el bus en `GPIO 8 (SDA)` y `GPIO 9 (SCL)` y activa el modo *Bypass*:

1. **MPU-6500 (Acelerómetro + Giróscopo):** Dirección `0x68`.
2. **AK8963 (Magnetómetro de 3 ejes):** Dirección `0x0C` (accesible en el mismo bus tras habilitar el registro `REG_INT_PIN_CFG = 0x37, bit 1 = 1`).

---

## 4. Recomendaciones de Montaje Físico

1. **Distancia a Metales y Cables de Alta Corriente:**
   El magnetómetro interno es sensible a tornillos ferromagnéticos, motores, imanes de altavoces o baterías de litio pegadas al sensor. Mantén el sensor a al menos 3-5 cm de masas metálicas ferrosas para asegurar una calibración magnética óptima.
2. **Orientación Física y Compensación 3D:**
   Si la IMU está rotada en la protoboard o dentro de una carcasa (por ejemplo 90° respecto a tu mano o control), no es necesario desoldar ni girar físicamente la placa. Utiliza el sistema de **Orientación de Montaje 3D (Yaw, Pitch, Roll)** en la interfaz web o el botón de **Detección Automática de 2 Gestos** para que el firmware remapee las rotaciones instantáneamente.
