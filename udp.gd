extends Node

## MPC transport.
##
## Native builds keep the original UDP behaviour:
##   * listen for 16-byte control packets on 127.0.0.1:4246
##   * send 5584-byte sensor packets to 127.0.0.1:4247
##
## Web builds use the in-page JavaScript controller by default (no network).
## Supplying ?ws=... or the exported `server_url` switches to WebSocket, where
## one binary message == one packet and the default URL is ws(s)://<page host>/ws.
##
## The public surface used by the rest of the project is unchanged:
##   steering_input, speed_input, send_data(buffer), group "network_transceivers".

const LISTEN_PORT := 4246
const DESTINATION_PORT := 4247
const DESTINATION_IP := "127.0.0.1"

const CONTROL_PACKET_SIZE := 16
const SENSOR_PACKET_SIZE := 5584

const RECONNECT_MIN := 0.5
const RECONNECT_MAX := 5.0

## Optional explicit WebSocket URL. Leave empty to derive it from the page URL.
@export var server_url := ""
## Print every control packet. Noisy at 200 Hz, useful for debugging.
@export var log_packets := false

var steering_input: float = 0.0
var speed_input: float = 0.0

var _udp: PacketPeerUDP = null
var _ws: WebSocketPeer = null
var _js: Object = null
var _mpc: Object = null
var _use_browser_mpc := false
var _is_web := false
var _ws_url := ""
var _reconnect_delay := RECONNECT_MIN
var _reconnect_timer := 0.0
var _last_ws_state := WebSocketPeer.STATE_CLOSED


func _ready() -> void:
	add_to_group("network_transceivers")

	_is_web = OS.has_feature("web")
	if _is_web:
		_js = Engine.get_singleton("JavaScriptBridge")
		_mpc = _get_browser_controller()
		if _mpc != null:
			_use_browser_mpc = true
			if _js != null:
				_js.eval("window.MPC_TRANSPORT = 'browser';", true)
			print("MPC transport: using in-browser JavaScript controller")
		else:
			if _js != null:
				_js.eval("window.MPC_TRANSPORT = 'websocket';", true)
			_ws_url = _resolve_ws_url()
			print("MPC transport: connecting to %s" % _ws_url)
			_connect_ws()
	else:
		_udp = PacketPeerUDP.new()
		var err := _udp.bind(LISTEN_PORT)
		if err != OK:
			push_error("MPC transport: could not bind UDP port %d (error %d)" % [LISTEN_PORT, err])


func _process(delta: float) -> void:
	if _is_web:
		if _use_browser_mpc:
			_poll_browser_mpc()
		else:
			_poll_websocket(delta)
		return

	if _udp == null:
		return

	while _udp.get_available_packet_count() > 0:
		_apply_control_packet(_udp.get_packet())


## Send one sensor packet to the controller.
## Signature kept identical to the original UDP implementation.
func send_data(buffer: PackedByteArray) -> void:
	if _use_browser_mpc:
		_submit_browser_mpc(buffer)
		return

	if _is_web:
		if _ws != null and _ws.get_ready_state() == WebSocketPeer.STATE_OPEN:
			var err := _ws.put_packet(buffer)
			if err != OK and log_packets:
				push_warning("MPC transport: WebSocket send failed (error %d)" % err)
		return

	if _udp == null:
		return

	_udp.set_dest_address(DESTINATION_IP, DESTINATION_PORT)
	_udp.put_packet(buffer)


func _apply_control_packet(packet: PackedByteArray) -> void:
	if packet.size() != CONTROL_PACKET_SIZE:
		return

	steering_input = packet.decode_double(0)
	speed_input = packet.decode_double(8)

	if log_packets:
		print("MPC control -> steer %.3f speed %.3f" % [steering_input, speed_input])


func _get_browser_controller() -> Object:
	if _js == null or _ws_override_present():
		return null
	return _js.get_interface("MPCController")


func _ws_override_present() -> bool:
	if server_url.strip_edges() != "":
		return true
	if _js == null:
		return false
	var from_query = _js.eval(
		"new URLSearchParams(window.location.search).get('ws') || ''", true
	)
	return from_query is String and from_query != ""


## Pull the latest browser-MPC control values. The JavaScript controller runs
## synchronously in the page, so no socket is involved.
func _poll_browser_mpc() -> void:
	if _mpc == null:
		return
	var steer = _mpc.call("getSteering")
	var speed = _mpc.call("getSpeed")
	if steer != null:
		steering_input = float(steer)
	if speed != null:
		speed_input = float(speed)


## Forward the 13 state values and 5 CTE values to the JavaScript controller.
## It returns steering and speed through getSteering()/getSpeed().
func _submit_browser_mpc(buffer: PackedByteArray) -> void:
	if _mpc == null or buffer.size() != SENSOR_PACKET_SIZE:
		return

	# A JSON string is used for the JavaScriptBridge call because it is
	# converted unambiguously to a JS string. A PackedFloat64Array should work
	# too, but the JSON path is stable across Godot Web builds.
	var values := []
	values.resize(18)
	for i in range(13):
		values[i] = buffer.decode_double(i * 8)
	for i in range(5):
		values[13 + i] = buffer.decode_double(5544 + i * 8)
	_mpc.call("submit", JSON.stringify(values))


func _resolve_ws_url() -> String:
	var explicit := server_url.strip_edges()
	if explicit != "":
		return explicit

	if _js != null:
		var from_query = _js.eval(
			"new URLSearchParams(window.location.search).get('ws') || ''", true
		)
		if from_query is String and from_query != "":
			return from_query

		var protocol = _js.eval("window.location.protocol", true)
		var host = _js.eval("window.location.host", true)
		if host is String and host != "":
			var scheme := "wss" if protocol == "https:" else "ws"
			return "%s://%s/ws" % [scheme, host]

	return "ws://127.0.0.1:8080/ws"


func _connect_ws() -> void:
	if _ws != null:
		_ws.close()
		_ws = null

	_ws = WebSocketPeer.new()
	var err := _ws.connect_to_url(_ws_url)
	if err != OK:
		push_warning("MPC transport: cannot connect to %s (error %d)" % [_ws_url, err])
		_ws = null
		_schedule_reconnect()


func _poll_websocket(delta: float) -> void:
	if _ws == null:
		_reconnect_timer -= delta
		if _reconnect_timer <= 0.0:
			_connect_ws()
		return

	_ws.poll()
	var state := _ws.get_ready_state()

	if state != _last_ws_state:
		_last_ws_state = state
		if state == WebSocketPeer.STATE_OPEN:
			_reconnect_delay = RECONNECT_MIN
			print("MPC transport: connected to %s" % _ws_url)
		elif state == WebSocketPeer.STATE_CLOSED and log_packets:
			print(
				"MPC transport: WebSocket closed (%d: %s)"
				% [_ws.get_close_code(), _ws.get_close_reason()]
			)

	match state:
		WebSocketPeer.STATE_OPEN:
			while _ws.get_available_packet_count() > 0:
				_apply_control_packet(_ws.get_packet())
		WebSocketPeer.STATE_CLOSED:
			_ws = null
			_schedule_reconnect()


func _schedule_reconnect() -> void:
	_reconnect_timer = _reconnect_delay
	_reconnect_delay = minf(_reconnect_delay * 1.5, RECONNECT_MAX)
