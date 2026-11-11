extends Node3D

@export var max_range := 50.0 # Maximum distance of the laser

var last_pos := Vector3.ZERO
var last_rot := Quaternion.IDENTITY
var send_timer := 0.0
var send_interval := 0.05 # 20 Hz transmission
var steer_history: Array[float] = [] # Storing steering samples

func _ready():
	# Ensure variables are initialized with current global state
	last_pos = global_position
	last_rot = global_transform.basis.get_rotation_quaternion()

const LOOKAHEAD_HORIZONS = [0.0, 2.5, 5.0, 7.5, 10.0]
var horizon_visuals: Array[MeshInstance3D] = []

func _physics_process(delta: float):
	if delta <= 0: return
	
	send_timer += delta
	if send_timer < send_interval:
		return
	send_timer = 0.0
	
	# --- 1. State Extraction ---
	var pos = global_position
	var curr_basis = global_transform.basis
	var quat = curr_basis.get_rotation_quaternion()
	
	# Calculate velocities
	var lin_vel = (pos - last_pos) / delta
	var q_diff = quat * last_rot.inverse()
	var ang_vel = Vector3(q_diff.x, q_diff.y, q_diff.z) * (2.0 / delta)
	
	last_pos = pos
	last_rot = quat

	# --- 2. 360 Degree Laser Scan ---
	var scan_data = []
	var space_state = get_world_3d().direct_space_state
	
	# Safety check for physics state
	if not space_state:
		return

	for i in range(360):
		var angle = deg_to_rad(i)
		# Create a direction vector (pointing out horizontally)
		var local_dir = Vector3(sin(angle), 0, cos(angle))
		
		# Rotate the local direction by the car's current global orientation
		var global_dir = curr_basis * local_dir 

		var ray_end = pos + global_dir * max_range
		var query = PhysicsRayQueryParameters3D.create(pos, ray_end)
		
		# Exclude the car itself to avoid self-collision
		var parent_node = get_parent()
		if parent_node is CollisionObject3D:
			query.exclude = [parent_node.get_rid()]
		
		var result = space_state.intersect_ray(query)
		if result:
			scan_data.append(pos.distance_to(result.position))
		else:
			scan_data.append(max_range)

	# --- 3. Ground Lane Scan (Synthetic/Physical Hybrid) ---
	var lane_data = []
	var cte_data = []
	var scan_width = 8.0 # meters
	var lane_res = 64
	var cte = 0.0
	
	# Ensure debug visuals exist
	if horizon_visuals.size() == 0:
		for i in range(LOOKAHEAD_HORIZONS.size()):
			var mi = MeshInstance3D.new()
			var pm = PlaneMesh.new()
			pm.size = Vector2(scan_width, 0.4)
			mi.mesh = pm
			var mat = StandardMaterial3D.new()
			mat.transparency = StandardMaterial3D.TRANSPARENCY_ALPHA
			mat.albedo_color = Color(0, 0.8, 1, 0.5) # Cyan 50%
			mat.shading_mode = StandardMaterial3D.SHADING_MODE_UNSHADED
			mi.material_override = mat
			mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			get_parent().get_parent().add_child(mi)
			horizon_visuals.append(mi)

	# Try to find the road under us
	var road_query = PhysicsRayQueryParameters3D.create(pos + Vector3.UP, pos + Vector3.DOWN * 5.0)
	road_query.collide_with_areas = true
	var road_hit = space_state.intersect_ray(road_query)
	
	var current_road_seg = null
	if road_hit:
		var p = road_hit.collider
		while p != null:
			if p.has_method("is_road_segment"):
				current_road_seg = p
				break
			p = p.get_parent()
	
	if current_road_seg:
		var local_car_p = current_road_seg.to_local(pos)
		var closest_off = current_road_seg.curve.get_closest_offset(local_car_p)
		
		# Get lane info
		var lane_width = 3.5
		var num_lanes = 2
		if current_road_seg.start_point:
			num_lanes = current_road_seg.start_point.lanes.size()
			lane_width = current_road_seg.start_point.lane_width
		var half_width = (num_lanes * lane_width) / 2.0
		
		for h_idx in range(LOOKAHEAD_HORIZONS.size()):
			var la = LOOKAHEAD_HORIZONS[h_idx]
			# Target position ahead of car (local -Z is forward)
			var target_pos = global_transform * Vector3(0, 0, -la)
			var local_target_p = current_road_seg.to_local(target_pos)
			var road_trans = current_road_seg.curve.sample_baked_with_rotation(closest_off + la)
			
			# Update Visual Plane
			var visual = horizon_visuals[h_idx]
			visual.global_position = target_pos + Vector3(0, 0.1, 0)
			visual.global_rotation = global_rotation
			
			# Calculate CTE for this horizon
			var to_car = local_target_p - road_trans.origin
			var current_cte = to_car.dot(road_trans.basis.x)
			cte_data.append(current_cte)
			if la == 0.0: cte = current_cte # base cte for hud
			
			# Generate intensity pattern
			for i in range(lane_res):
				var t = float(i) / (lane_res - 1)
				var scan_offset = lerp(-scan_width/2.0, scan_width/2.0, t)
				var lateral_dist = current_cte + scan_offset
				var intensity = 0.1
				var dist_from_center = abs(lateral_dist)
				var mod_dist = fmod(dist_from_center + lane_width/2.0, lane_width)
				
				if dist_from_center > half_width + 0.1: intensity = 0.0
				elif abs(mod_dist - lane_width/2.0) < 0.15: intensity = 1.0
				elif abs(dist_from_center - 0.0) < 0.2 and num_lanes % 2 == 0: intensity = 1.0
				elif abs(dist_from_center - half_width) < 0.15: intensity = 0.8
				else: intensity = 0.2
				
				lane_data.append(intensity)
	else:
		# Failover if no road found
		for _la in range(LOOKAHEAD_HORIZONS.size()):
			cte_data.append(0.0)
			for i in range(lane_res): lane_data.append(0.0)

	# --- 4. Encoding (698 doubles = 5584 bytes) ---
	var buffer = PackedByteArray()
	buffer.resize(698 * 8) 
	
	var state_array = [
		pos.x, pos.y, pos.z, 
		quat.w, quat.x, quat.y, quat.z, 
		lin_vel.x, lin_vel.y, lin_vel.z, 
		ang_vel.x, ang_vel.y, ang_vel.z
	]
	
	# Part A: State (13 doubles)
	for i in range(state_array.size()):
		buffer.encode_double(i * 8, state_array[i])
		
	# Part B: Lidar (360 doubles) starting at byte 104
	for i in range(scan_data.size()):
		buffer.encode_double(104 + (i * 8), scan_data[i])
		
	# Part C: Lane Intensity Flattened (320 doubles) starting at byte 2984
	for i in range(lane_data.size()):
		buffer.encode_double(2984 + (i * 8), lane_data[i])
		
	# Part D: CTE Array (5 doubles) starting at byte 5544
	for i in range(cte_data.size()):
		buffer.encode_double(5544 + (i * 8), cte_data[i])

	# --- 5. Update HUD Overlay ---
	var current_steer = 0.0
	var transport: Node = null
	for node in get_tree().get_nodes_in_group("network_transceivers"):
		if node.has_method("send_data"):
			transport = node
			break

	if transport and "steering_input" in transport:
		current_steer = transport.steering_input
		steer_history.append(current_steer)
		if steer_history.size() > 1000: steer_history.pop_front()
		
	_update_hud(state_array, scan_data, lane_data.slice(0, 64), cte, current_steer)

	# --- 6. Send to transport (UDP on desktop, WebSocket on web) ---
	if transport:
		transport.send_data(buffer)

var hud_instance: CanvasLayer = null

func _update_hud(state, lidar, lane, current_cte, steer):
	if hud_instance == null:
		var hud_script = load("res://hud_overlay.gd")
		if hud_script:
			hud_instance = hud_script.new()
			add_child(hud_instance)
	
	if hud_instance:
		hud_instance.state_data = state
		hud_instance.lidar_data = lidar
		hud_instance.lane_data = lane
		hud_instance.cte = current_cte
		hud_instance.steer = steer
