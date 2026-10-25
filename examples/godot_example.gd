# Motion Controller Lab — Script de Integración para Godot 4 (GDScript)
# ====================================================================
# Adjunta este script a cualquier Node3D (por ejemplo, una raqueta, espada,
# avión o control) para animarlo con los movimientos del ESP32-C3 en tiempo real.
#
# URL por defecto: ws://127.0.0.1:8765

extends Node3D

@export var websocket_url: String = "ws://127.0.0.1:8765"
@export var smooth_factor: float = 25.0 # Suavizado Slerp para 60+ FPS
@export var invert_yaw: bool = true

var _socket: WebSocketPeer = WebSocketPeer.new()
var _target_quat: Quaternion = Quaternion.IDENTITY
var _tare_quat: Quaternion = Quaternion.IDENTITY
var _is_connected: bool = false


func _ready() -> void:
	print("[MotionController] Conectando a ", websocket_url)
	var err = _socket.connect_to_url(websocket_url)
	if err != OK:
		push_error("Error al iniciar conexión WebSocket: %d" % err)


func _process(delta: float) -> void:
	_socket.poll()
	var state = _socket.get_ready_state()

	if state == WebSocketPeer.STATE_OPEN:
		if not _is_connected:
			_is_connected = true
			print("[MotionController] ✓ Conectado con éxito al Motion Gateway!")

		while _socket.get_available_packet_count() > 0:
			var packet = _socket.get_packet()
			_handle_json_packet(packet.get_string_from_utf8())

	elif state == WebSocketPeer.STATE_CLOSED:
		if _is_connected:
			_is_connected = false
			print("[MotionController] Conexión cerrada. Reconectando...")
		# Reintento de reconexión tras breve espera
		_socket.connect_to_url(websocket_url)

	# Tecla Espacio para centrar / Tare (neutralizar posición actual)
	if Input.is_action_just_pressed("ui_accept"):
		tare()

	# Interpolar suavemente hacia la orientación objetivo
	quaternion = quaternion.slerp(_tare_quat * _target_quat, smooth_factor * delta)


func _handle_json_packet(json_str: String) -> void:
	var json = JSON.new()
	var parse_err = json.parse(json_str)
	if parse_err != OK:
		return

	var data = json.data
	if typeof(data) != TYPE_DICTIONARY or data.get("type") != "telemetry":
		return

	var q = data.get("quaternion")
	if q != null:
		var qw: float = float(q.get("w", 1.0))
		var qx: float = float(q.get("x", 0.0))
		var qy: float = float(q.get("y", 0.0))
		var qz: float = float(q.get("z", 0.0))

		# Mapeo de convención de ejes IMU (Z arriba, Y adelante, X derecha)
		# a convención de Godot 3D (Y arriba, -Z adelante, X derecha):
		var godot_x = qx
		var godot_y = qz * (-1.0 if invert_yaw else 1.0)
		var godot_z = -qy
		var godot_w = qw

		_target_quat = Quaternion(godot_x, godot_y, godot_z, godot_w).normalized()


## Calibra la postura actual como punto cero (Tare)
func tare() -> void:
	_tare_quat = _target_quat.inverse()
	print("[MotionController] Tare realizado: Postura neutral capturada")
