"""
Motion Controller Lab — Cliente de Ejemplo en Python
===================================================
Se conecta al Gateway en ws://127.0.0.1:8765 y recibe la orientación
y cuaterniones a 50 Hz en tiempo real.

Requisitos:
    pip install websockets

Uso:
    python python_client.py
"""

import asyncio
import json
import math
import sys

GATEWAY_WS_URL = "ws://127.0.0.1:8765"


async def run_motion_client():
    try:
        import websockets
    except ImportError:
        print("[ERROR] La librería 'websockets' no está instalada.")
        print("Instálala ejecutando: pip install websockets")
        return

    print(f"[*] Conectando con Motion Gateway en {GATEWAY_WS_URL}...")

    while True:
        try:
            async with websockets.connect(GATEWAY_WS_URL) as ws:
                print("[✓] Conexión establecida. Recibiendo telemetría a 50 Hz...\n")
                
                # Encabezado para consola
                print(f"{'SEQ':>6} | {'ROLL':>8} {'PITCH':>8} {'YAW_REL':>8} {'YAW_MAG':>8} | {'QUAT (W, X, Y, Z)':>28} | {'ESTADO'}")
                print("-" * 88)

                sample_count = 0
                async for message in ws:
                    try:
                        data = json.loads(message)
                    except json.JSONDecodeError:
                        continue

                    msg_type = data.get("type")
                    if msg_type != "telemetry":
                        continue

                    sample_count += 1
                    seq = data.get("sequence", 0)
                    ori = data.get("orientation", {})
                    q = data.get("quaternion")

                    roll = ori.get("roll_deg")
                    pitch = ori.get("pitch_deg")
                    yaw = ori.get("yaw_rel_deg")
                    yaw_mag = ori.get("yaw_mag_rel_deg")

                    r_str = f"{roll:>7.1f}°" if roll is not None else "    N/A "
                    p_str = f"{pitch:>7.1f}°" if pitch is not None else "    N/A "
                    y_str = f"{yaw:>7.1f}°" if yaw is not None else "    N/A "
                    ym_str = f"{yaw_mag:>7.1f}°" if yaw_mag is not None else "    N/A "

                    if q:
                        q_str = f"({q['w']:>5.2f}, {q['x']:>5.2f}, {q['y']:>5.2f}, {q['z']:>5.2f})"
                    else:
                        q_str = "(       Sin Quat       )"

                    # Imprimir en consola cada 5 muestras para no saturar la terminal (10 Hz en pantalla)
                    if sample_count % 5 == 0:
                        sys.stdout.write(f"\r{seq:>6} | {r_str} {p_str} {y_str} {ym_str} | {q_str} | ACTIVO ")
                        sys.stdout.flush()

        except (websockets.exceptions.ConnectionClosedError, OSError):
            print("\n[!] Conexión perdida o servidor no disponible. Reintentando en 2 segundos...")
            await asyncio.sleep(2.0)
        except asyncio.CancelledError:
            print("\n[*] Cliente finalizado.")
            break


if __name__ == "__main__":
    try:
        asyncio.run(run_motion_client())
    except KeyboardInterrupt:
        print("\n[*] Saliendo...")
