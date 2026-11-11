extends Node3D

# We will look for the transport node to get commands
# If not using Autoload, we find it in the tree
@onready var udp_node = _find_transport()

func _find_transport():
	for node in get_tree().get_nodes_in_group("network_transceivers"):
		if node.has_method("send_data"):
			return node
	return null

var trail_points = []
var trail_mesh_instance: MeshInstance3D
var trail_mesh: ImmediateMesh

var is_top_down := false
@onready var camera = get_node_or_null("Camera3D")
@onready var initial_cam_pos = camera.position if camera else Vector3.ZERO
@onready var initial_cam_rot = camera.rotation if camera else Vector3.ZERO

func _input(event):
	if event.is_action_pressed("ui_c") or (event is InputEventKey and event.pressed and event.keycode == KEY_C):
		is_top_down = !is_top_down
		_update_camera_view()

func _physics_process(delta: float):
	var steer = 0.0
	var target_speed = 0.0
	
	# 1. Get values from the UDP node if it exists
	if udp_node:
		steer = udp_node.steering_input
		target_speed = udp_node.speed_input
	
	# 2. Apply Steering (Rotate the Node3D)
	rotate_y(steer * delta)
	
	# 3. Apply Forward Movement
	var forward_vector = -global_transform.basis.z
	global_position += forward_vector * target_speed * delta
	
	# Trail Logic
	if Engine.get_frames_drawn() % 3 == 0:
		trail_points.append(global_position)
		_update_trail_mesh()
	
	if is_top_down:
		_update_camera_view() # Keep follow in top down

func _update_camera_view():
	if not camera: return
	if is_top_down:
		camera.top_level = true
		camera.global_position = global_position + Vector3(0, 40, 0)
		camera.global_rotation_degrees = Vector3(-90, 0, 0)
	else:
		camera.top_level = false
		camera.position = initial_cam_pos
		camera.rotation = initial_cam_rot

func _update_trail_mesh():
	if trail_points.size() < 2: return
	
	if not trail_mesh_instance:
		trail_mesh_instance = MeshInstance3D.new()
		get_parent().add_child(trail_mesh_instance)
		var mat = StandardMaterial3D.new()
		mat.albedo_color = Color.YELLOW
		mat.emission_enabled = true
		mat.emission = Color.YELLOW
		trail_mesh_instance.material_override = mat
		trail_mesh = ImmediateMesh.new()
		trail_mesh_instance.mesh = trail_mesh
		
	trail_mesh.clear_surfaces()
	trail_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in trail_points:
		trail_mesh.surface_add_vertex(p + Vector3(0, 0.2, 0)) # slightly above ground
	trail_mesh.surface_end()
