extends Node3D
## Load robot.json, draw pivot/axis debug, let the user manually drive each
## joint to test pivots (CadRobotLoader contract: this script never computes
## kinematics, only reads what the pipeline already resolved).

const ROBOT_JSON := "res://robot_data/robot.json"
const OVERLAY_JSON := "res://robot_data/debug_overlay.json"
const LOADER_SCRIPT := preload("res://addons/cad_robot_importer/robot_loader.gd")

const REVOLUTE_LIMIT := PI          # +-180 deg, generic (robot.json carries no limits yet)
const PRISMATIC_LIMIT := 0.15       # +-15cm, generic
const REVOLUTE_SPEED := 1.2         # rad/s while key held
const PRISMATIC_SPEED := 0.08       # m/s while key held

var _robot: Node3D
var _joint_nodes: Array[Node3D] = []
var _joint_angles: Array[float] = []
var _selected := 0
var _tint_on := false
var _overlay_data: Dictionary = {}
var _hud: Label

var _cam_target := Vector3.ZERO
var _cam_yaw := -0.7
var _cam_pitch := 0.5
var _cam_dist := 1.0
var _cam_min_dist := 0.05


func _ready() -> void:
	_robot = LOADER_SCRIPT.load_robot(ROBOT_JSON)
	if _robot == null:
		push_error("Robot yüklenemedi: %s" % ROBOT_JSON)
		return
	add_child(_robot)
	_collect_joints(_robot)
	_load_overlay_data()
	_build_debug_overlay()
	_build_hud()
	print("Robot yüklendi. Hareketli joint: ", _joint_nodes.size())
	_fit_camera()
	_runtime_smoke()
	_update_hud()


func _collect_joints(node: Node) -> void:
	if node.has_meta("joint_type"):
		var t := str(node.get_meta("joint_type"))
		if t == "revolute" or t == "prismatic":
			_joint_nodes.append(node as Node3D)
	for c in node.get_children():
		_collect_joints(c)
	if node == _robot:
		_joint_angles.resize(_joint_nodes.size())
		_joint_angles.fill(0.0)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		var motion := event as InputEventMouseMotion
		_cam_yaw -= motion.relative.x * 0.005
		_cam_pitch = clamp(_cam_pitch - motion.relative.y * 0.005, -1.4, 1.4)
		_update_camera()
	elif event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if not mb.pressed:
			return
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			_cam_dist = max(_cam_dist * 0.9, _cam_min_dist)
			_update_camera()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_cam_dist *= 1.1
			_update_camera()


func _unhandled_key_input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return
	var key_event := event as InputEventKey
	if not key_event.pressed or key_event.echo:
		return
	var key := key_event.keycode
	if _joint_nodes.is_empty():
		return
	if key == KEY_TAB:
		if key_event.shift_pressed:
			_selected = (_selected - 1 + _joint_nodes.size()) % _joint_nodes.size()
		else:
			_selected = (_selected + 1) % _joint_nodes.size()
		_update_hud()
	elif key == KEY_R:
		_joint_angles[_selected] = 0.0
		LOADER_SCRIPT.set_joint(_joint_nodes[_selected], 0.0)
		_update_hud()
	elif key == KEY_SPACE:
		for i in _joint_nodes.size():
			_joint_angles[i] = 0.0
			LOADER_SCRIPT.set_joint(_joint_nodes[i], 0.0)
		_update_hud()
	elif key == KEY_C:
		_tint_on = not _tint_on
		_apply_tint(_robot, _tint_on)
		_update_hud()


func _process(delta: float) -> void:
	if _joint_nodes.is_empty():
		return
	var node := _joint_nodes[_selected]
	var jtype := str(node.get_meta("joint_type"))
	var limit := REVOLUTE_LIMIT if jtype == "revolute" else PRISMATIC_LIMIT
	var speed := REVOLUTE_SPEED if jtype == "revolute" else PRISMATIC_SPEED
	var dir := 0.0
	if Input.is_physical_key_pressed(KEY_LEFT) or Input.is_physical_key_pressed(KEY_A):
		dir -= 1.0
	if Input.is_physical_key_pressed(KEY_RIGHT) or Input.is_physical_key_pressed(KEY_D):
		dir += 1.0
	if dir != 0.0:
		_joint_angles[_selected] = clamp(_joint_angles[_selected] + dir * speed * delta, -limit, limit)
		LOADER_SCRIPT.set_joint(node, _joint_angles[_selected])
		_update_hud()


func _load_overlay_data() -> void:
	if not FileAccess.file_exists(OVERLAY_JSON):
		return
	var f := FileAccess.open(OVERLAY_JSON, FileAccess.READ)
	if f == null:
		return
	var data = JSON.parse_string(f.get_as_text())
	if typeof(data) == TYPE_DICTIONARY:
		_overlay_data = data


func _apply_tint(node: Node, on: bool) -> void:
	var colors: Dictionary = _overlay_data.get("link_colors", {})
	if node is Node3D and colors.has(node.name):
		for c in node.get_children():
			if c is MeshInstance3D:
				var mi := c as MeshInstance3D
				if mi.mesh == null:
					continue
				if not on:
					for si in range(mi.mesh.get_surface_count()):
						mi.set_surface_override_material(si, null)
					continue
				var rgb: Array = colors[node.name].get("rgb", [0.8, 0.8, 0.8])
				var mat := StandardMaterial3D.new()
				mat.albedo_color = Color(float(rgb[0]), float(rgb[1]), float(rgb[2]))
				for si in range(mi.mesh.get_surface_count()):
					mi.set_surface_override_material(si, mat)
	for c in node.get_children():
		_apply_tint(c, on)


func _build_debug_overlay() -> void:
	var debug := Node3D.new()
	debug.name = "DebugOverlay"
	add_child(debug)
	if _overlay_data.is_empty():
		push_warning("debug_overlay.json yok — pipeline validate çalıştır")
		return
	for m in _overlay_data.get("markers", []):
		var pos: Array = m.get("position", [0, 0, 0])
		var rgb: Array = m.get("color", [1, 1, 0])
		var mesh := SphereMesh.new()
		mesh.radius = float(m.get("radius_m", 0.012))
		mesh.height = mesh.radius * 2.0
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color(float(rgb[0]), float(rgb[1]), float(rgb[2]))
		mat.emission_enabled = true
		mat.emission = mat.albedo_color
		mat.emission_energy_multiplier = 0.6
		mi.material_override = mat
		mi.position = Vector3(pos[0], pos[1], pos[2])
		# Parent under robot so CAD Z-up root rotation applies if any
		_robot.add_child(mi)
	for ax in _overlay_data.get("axes", []):
		var a: Array = ax.get("a", [0, 0, 0])
		var b: Array = ax.get("b", [0, 0, 1])
		var rgb: Array = ax.get("color", [1, 1, 0])
		var im := ImmediateMesh.new()
		var mi := MeshInstance3D.new()
		mi.mesh = im
		var mat := StandardMaterial3D.new()
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		mat.albedo_color = Color(float(rgb[0]), float(rgb[1]), float(rgb[2]))
		mi.material_override = mat
		im.surface_begin(Mesh.PRIMITIVE_LINES, mat)
		im.surface_add_vertex(Vector3(a[0], a[1], a[2]))
		im.surface_add_vertex(Vector3(b[0], b[1], b[2]))
		im.surface_end()
		_robot.add_child(mi)


func _build_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	_hud = Label.new()
	_hud.position = Vector2(12, 12)
	_hud.add_theme_font_size_override("font_size", 16)
	_hud.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.8))
	_hud.add_theme_constant_override("shadow_offset_x", 1)
	_hud.add_theme_constant_override("shadow_offset_y", 1)
	layer.add_child(_hud)


func _update_hud() -> void:
	if _hud == null:
		return
	if _joint_nodes.is_empty():
		_hud.text = "robot.json içinde hareketli joint yok."
		return
	var node := _joint_nodes[_selected]
	var jtype := str(node.get_meta("joint_type"))
	var jid := str(node.get_meta("joint_id"))
	var val := _joint_angles[_selected]
	var val_str := ("%.1f°" % rad_to_deg(val)) if jtype == "revolute" else ("%.3f m" % val)
	_hud.text = (
		"Joint %d/%d — %s (%s)  parent→%s\n"
		+ "Değer: %s\n\n"
		+ "TAB / Shift+TAB: joint seç   ←/→ (A/D basılı tut): döndür\n"
		+ "R: seçili joint'i sıfırla   SPACE: hepsini sıfırla   C: renk overlay aç/kapa\n"
		+ "Sağ tık + sürükle: kamerayı döndür   Scroll: yakınlaş/uzaklaş"
	) % [_selected + 1, _joint_nodes.size(), jid, jtype, str(node.get_parent().name), val_str]


func _runtime_smoke() -> void:
	var fails := 0
	for n in _joint_nodes:
		var rest := n.global_position
		LOADER_SCRIPT.set_joint(n, 0.2)
		var after := n.global_position
		LOADER_SCRIPT.set_joint(n, 0.0)
		var drift := rest.distance_to(after)
		var jtype := str(n.get_meta("joint_type"))
		if jtype == "revolute" and drift > 1e-4:
			push_error("Pivot drift on %s: %s" % [n.name, drift])
			fails += 1
		elif jtype == "prismatic" and drift < 1e-6:
			push_error("Prismatic no motion on %s" % n.name)
			fails += 1
	if fails == 0:
		print("Godot runtime smoke OK (", _joint_nodes.size(), " joints)")
	else:
		push_error("Godot runtime smoke FAIL count=%s" % fails)


func _fit_camera() -> void:
	if _robot == null:
		return
	var bounds := _collect_aabb(_robot, AABB())
	var center := bounds.position + bounds.size * 0.5
	var radius := bounds.size.length() * 0.5
	if radius <= 0.0:
		radius = 0.3
	_cam_target = center
	_cam_dist = radius * 2.5
	_cam_min_dist = radius * 0.15
	_update_camera()


func _collect_aabb(node: Node, acc: AABB) -> AABB:
	if node is MeshInstance3D and node.mesh != null:
		var mi := node as MeshInstance3D
		var world_aabb: AABB = mi.global_transform * mi.get_aabb()
		acc = world_aabb if acc.size == Vector3.ZERO else acc.merge(world_aabb)
	for c in node.get_children():
		acc = _collect_aabb(c, acc)
	return acc


func _update_camera() -> void:
	var cam := $Camera3D as Camera3D
	if cam == null:
		return
	var offset := Vector3(
		cos(_cam_pitch) * sin(_cam_yaw),
		sin(_cam_pitch),
		cos(_cam_pitch) * cos(_cam_yaw)
	) * _cam_dist
	cam.global_position = _cam_target + offset
	cam.look_at(_cam_target, Vector3.UP)
