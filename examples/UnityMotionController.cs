// Motion Controller Lab — Script de Integración para Unity 3D (C#)
// ==============================================================
// Adjunta este script a cualquier GameObject (mando, espada, raqueta, dron)
// para sincronizar su orientación en tiempo real con el ESP32-C3 vía WebSocket.
//
// Puedes usar NativeWebSocket (https://github.com/endel/NativeWebSocket)
// o cualquier librería de WebSocket compatible con Unity.

using System;
using System.Text;
using UnityEngine;

#if USE_NATIVE_WEBSOCKET
using NativeWebSocket;
#endif

namespace MotionControllerLab
{
    [Serializable]
    public class QuaternionData
    {
        public float w = 1f;
        public float x = 0f;
        public float y = 0f;
        public float z = 0f;
    }

    [Serializable]
    public class OrientationData
    {
        public float roll_deg;
        public float pitch_deg;
        public float yaw_rel_deg;
        public float yaw_mag_rel_deg;
    }

    [Serializable]
    public class TelemetryMessage
    {
        public string type;
        public int sequence;
        public OrientationData orientation;
        public QuaternionData quaternion;
    }

    public class UnityMotionController : MonoBehaviour
    {
        [Header("Configuración WebSocket")]
        [Tooltip("Dirección del Gateway local")]
        public string websocketUrl = "ws://127.0.0.1:8765";

        [Header("Ajustes de Orientación")]
        [Range(1f, 50f)]
        public float slerpSpeed = 25f;
        public bool invertYaw = true;

        private Quaternion _targetRotation = Quaternion.identity;
        private Quaternion _tareRotation = Quaternion.identity;

#if USE_NATIVE_WEBSOCKET
        private WebSocket _websocket;

        private async void Start()
        {
            _websocket = new WebSocket(websocketUrl);

            _websocket.OnOpen += () =>
            {
                Debug.Log("[MotionController] ✓ Conectado al Gateway WebSocket!");
            };

            _websocket.OnError += (e) =>
            {
                Debug.LogError("[MotionController] Error WebSocket: " + e);
            };

            _websocket.OnClose += (e) =>
            {
                Debug.LogWarning("[MotionController] Conexión WebSocket cerrada.");
            };

            _websocket.OnMessage += (bytes) =>
            {
                string jsonString = Encoding.UTF8.GetString(bytes);
                ProcessTelemetry(jsonString);
            };

            await _websocket.Connect();
        }

        private void Update()
        {
#if !UNITY_WEBGL || UNITY_EDITOR
            _websocket?.DispatchMessageQueue();
#endif

            // Tecla Espacio para Tare (Centrar)
            if (Input.GetKeyDown(KeyCode.Space))
            {
                Tare();
            }

            // Aplicar rotación suave con interpolación Slerp
            transform.rotation = Quaternion.Slerp(transform.rotation, _tareRotation * _targetRotation, Time.deltaTime * slerpSpeed);
        }

        private async void OnApplicationQuit()
        {
            if (_websocket != null)
            {
                await _websocket.Close();
            }
        }
#else
        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                Tare();
            }
            transform.rotation = Quaternion.Slerp(transform.rotation, _tareRotation * _targetRotation, Time.deltaTime * slerpSpeed);
        }
#endif

        /// <summary>
        /// Procesa el mensaje JSON recibido del Gateway y convierte coordenadas de IMU a Unity.
        /// </summary>
        public void ProcessTelemetry(string jsonString)
        {
            try
            {
                TelemetryMessage msg = JsonUtility.FromJson<TelemetryMessage>(jsonString);
                if (msg != null && msg.type == "telemetry" && msg.quaternion != null)
                {
                    // Conversión de sistema de coordenadas de la IMU (Z arriba, Y adelante, X derecha)
                    // al sistema de coordenadas de Unity (Y arriba, Z adelante, X derecha):
                    float unityX = msg.quaternion.x;
                    float unityY = msg.quaternion.z * (invertYaw ? -1f : 1f);
                    float unityZ = -msg.quaternion.y;
                    float unityW = msg.quaternion.w;

                    _targetRotation = new Quaternion(unityX, unityY, unityZ, unityW);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[MotionController] Error procesando JSON: " + ex.Message);
            }
        }

        /// <summary>
        /// Establece la postura actual como origen (Tare).
        /// </summary>
        public void Tare()
        {
            _tareRotation = Quaternion.Inverse(_targetRotation);
            Debug.Log("[MotionController] Tare realizado (Postura neutral fijada)");
        }
    }
}
