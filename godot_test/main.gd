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

## Test Motion: a pivot-verification sweep, not a stress test. Exactly one
## joint moves at a time — everything else (including its own descendants,
## which ride along rigidly, exactly as a real robot would) stays at rest —
## so there is no cross-joint compounding and nothing to collide with that
## wasn't already adjacent in the rest pose.
const ANIMATE_JOINT_DURATION := 2.2  # seconds for one out-and-back sweep
const ANIMATE_FRACTION := 0.6        # peak sweep, as a fraction of the safe range

var _robot: Node3D
var _robot_root: Node3D  # RobotRoot — carries the CAD->Y-up conversion; _robot itself never does
var _joint_nodes: Array[Node3D] = []
var _joint_angles: Array[float] = []
var _selected := 0
var _tint_on := false
var _overlay_data: Dictionary = {}

var _animate_all := false
var _animate_joint_idx := 0
var _animate_local_t := 0.0

var _cam_target := Vector3.ZERO
var _cam_yaw := -0.7
var _cam_pitch := 0.5
var _cam_dist := 1.0
var _cam_min_dist := 0.05
var _robot_aabb := AABB()

# --- panel ---
var _panel_body: VBoxContainer
var _collapsed := false
var _info_label: Label
var _motion_status_label: Label
var _motion_btn: Button
var _collapse_btn: Button


func _ready() -> void:
	_robot = LOADER_SCRIPT.load_robot(ROBOT_JSON)
	if _robot == null:
		push_error("Robot yüklenemedi: %s" % ROBOT_JSON)
		return
	add_child(_robot)
	_robot_root = _robot.get_node("RobotRoot")
	_collect_joints(_robot)
	_load_overlay_data()
	_build_debug_overlay()
	_build_panel()
	print("Robot yüklendi. Hareketli joint: ", _joint_nodes.size())
	_fit_camera()
	_build_ground()
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
			_select_prev()
		else:
			_select_next()
	elif key == KEY_R:
		_reset_selected()
	elif key == KEY_SPACE:
		_reset_all()
	elif key == KEY_C:
		_toggle_tint()
	elif key == KEY_M:
		_toggle_animate_all()


func _select_next() -> void:
	_selected = (_selected + 1) % _joint_nodes.size()
	_update_hud()


func _select_prev() -> void:
	_selected = (_selected - 1 + _joint_nodes.size()) % _joint_nodes.size()
	_update_hud()


func _reset_selected() -> void:
	_joint_angles[_selected] = 0.0
	LOADER_SCRIPT.set_joint(_joint_nodes[_selected], 0.0)
	_update_hud()


func _reset_all() -> void:
	for i in _joint_nodes.size():
		_joint_angles[i] = 0.0
		LOADER_SCRIPT.set_joint(_joint_nodes[i], 0.0)
	_update_hud()


func _toggle_tint() -> void:
	_tint_on = not _tint_on
	_apply_tint(_robot, _tint_on)
	_update_hud()


func _process(delta: float) -> void:
	if _joint_nodes.is_empty():
		return

	if _animate_all:
		_animate_local_t += delta
		if _animate_local_t >= ANIMATE_JOINT_DURATION:
			LOADER_SCRIPT.set_joint(_joint_nodes[_animate_joint_idx], 0.0)
			_animate_local_t = 0.0
			_animate_joint_idx = (_animate_joint_idx + 1) % _joint_nodes.size()
			_selected = _animate_joint_idx
		var n := _joint_nodes[_animate_joint_idx]
		var jt := str(n.get_meta("joint_type"))
		var lim := REVOLUTE_LIMIT if jt == "revolute" else PRISMATIC_LIMIT
		# One smooth hump: 0 -> peak -> 0 across the duration. Never a
		# discontinuity, never past the same safe fraction manual control
		# already respects, and only this one joint's subtree is moving.
		var progress := _animate_local_t / ANIMATE_JOINT_DURATION
		var val := sin(progress * PI) * lim * ANIMATE_FRACTION
		LOADER_SCRIPT.set_joint(n, val)
		_update_hud()
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
		var rgb: Array = colors[node.name].get("rgb", [0.8, 0.8, 0.8])
		var mat: StandardMaterial3D = null
		if on:
			mat = StandardMaterial3D.new()
			mat.albedo_color = Color(float(rgb[0]), float(rgb[1]), float(rgb[2]))
		_tint_mesh_instances(node, mat)
	for c in node.get_children():
		_apply_tint(c, on)


## Mesh instances live inside the instantiated GLB scene subtree, at whatever
## depth the exporter's node hierarchy put them — not necessarily a direct
## child of the link node, so this has to recurse.
func _tint_mesh_instances(node: Node, mat: StandardMaterial3D) -> void:
	if node is MeshInstance3D:
		var mi := node as MeshInstance3D
		if mi.mesh != null:
			for si in range(mi.mesh.get_surface_count()):
				mi.set_surface_override_material(si, mat)
	for c in node.get_children():
		_tint_mesh_instances(c, mat)


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
		# Parent under RobotRoot (not _robot itself) so the CAD Z-up -> Y-up
		# conversion applies to these raw cad-frame positions too.
		_robot_root.add_child(mi)
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
		_robot_root.add_child(mi)


## Anchored top-right, fixed width, one control per row — stays put and
## legible at any window size, and collapses to just its header when it's
## in the way of the viewport.
func _build_panel() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)

	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	panel.offset_left = -336
	panel.offset_right = -12
	panel.offset_top = 12
	panel.custom_minimum_size = Vector2(324, 0)
	layer.add_child(panel)

	var outer := VBoxContainer.new()
	outer.add_theme_constant_override("separation", 8)
	panel.add_child(outer)

	var header := HBoxContainer.new()
	outer.add_child(header)
	var title := Label.new()
	title.text = "CAD Robot Test"
	title.add_theme_font_size_override("font_size", 15)
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(title)
	_collapse_btn = Button.new()
	_collapse_btn.text = "▾"
	_collapse_btn.custom_minimum_size = Vector2(28, 28)
	_collapse_btn.pressed.connect(_toggle_collapsed)
	header.add_child(_collapse_btn)

	_panel_body = VBoxContainer.new()
	_panel_body.add_theme_constant_override("separation", 6)
	outer.add_child(_panel_body)

	_info_label = Label.new()
	_info_label.autowrap_mode = TextServer.AUTOWRAP_WORD
	_panel_body.add_child(_info_label)

	_panel_body.add_child(HSeparator.new())

	_add_row_button("◀ Önceki Joint", _select_prev)
	_add_row_button("Sonraki Joint ▶", _select_next)
	_add_row_button("Seçili Joint'i Sıfırla (R)", _reset_selected)
	_add_row_button("Tümünü Sıfırla (SPACE)", _reset_all)
	_add_row_button("Renk Overlay Aç/Kapa (C)", _toggle_tint)

	_panel_body.add_child(HSeparator.new())

	_motion_status_label = Label.new()
	_motion_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD
	_panel_body.add_child(_motion_status_label)

	_motion_btn = Button.new()
	_motion_btn.text = "Test Motion Başlat (M)"
	_motion_btn.custom_minimum_size = Vector2(0, 32)
	_motion_btn.toggle_mode = true
	_motion_btn.pressed.connect(_toggle_animate_all)
	_panel_body.add_child(_motion_btn)

	_panel_body.add_child(HSeparator.new())

	var hint := Label.new()
	hint.text = (
		"←/→ (A/D basılı tut): seçili joint'i döndür\n"
		+ "Sağ tık + sürükle: kamerayı döndür\n"
		+ "Scroll: yakınlaş/uzaklaş"
	)
	hint.add_theme_font_size_override("font_size", 12)
	hint.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
	hint.autowrap_mode = TextServer.AUTOWRAP_WORD
	_panel_body.add_child(hint)


func _add_row_button(text: String, handler: Callable) -> void:
	var btn := Button.new()
	btn.text = text
	btn.custom_minimum_size = Vector2(0, 30)
	btn.pressed.connect(handler)
	_panel_body.add_child(btn)


func _toggle_collapsed() -> void:
	_collapsed = not _collapsed
	_panel_body.visible = not _collapsed
	_collapse_btn.text = "▸" if _collapsed else "▾"


func _toggle_animate_all() -> void:
	_animate_all = not _animate_all
	_motion_btn.button_pressed = _animate_all
	_motion_btn.text = "Test Motion Durdur (M)" if _animate_all else "Test Motion Başlat (M)"
	if _animate_all:
		_animate_joint_idx = 0
		_animate_local_t = 0.0
	else:
		# Hand back exactly where manual per-joint control left off, not a snap to 0.
		for i in _joint_nodes.size():
			LOADER_SCRIPT.set_joint(_joint_nodes[i], _joint_angles[i])
	_update_hud()


func _update_hud() -> void:
	if _info_label == null:
		return
	if _joint_nodes.is_empty():
		_info_label.text = "robot.json içinde hareketli joint yok."
		_motion_status_label.text = ""
		return
	var node := _joint_nodes[_selected]
	var jtype := str(node.get_meta("joint_type"))
	var jid := str(node.get_meta("joint_id"))
	var val := _joint_angles[_selected]
	var val_str := ("%.1f°" % rad_to_deg(val)) if jtype == "revolute" else ("%.3f m" % val)
	_info_label.text = (
		"Joint %d/%d — %s (%s)\nparent → %s\nDeğer: %s"
	) % [_selected + 1, _joint_nodes.size(), jid, jtype, str(node.get_parent().name), val_str]

	if _animate_all:
		var active := _joint_nodes[_animate_joint_idx]
		_motion_status_label.text = "Test ediliyor: %d/%d — %s" % [
			_animate_joint_idx + 1, _joint_nodes.size(), str(active.get_meta("joint_id"))
		]
	else:
		_motion_status_label.text = "Test Motion: kapalı — her joint'i sırayla, tek başına, güvenli aralıkta sallar"


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
	_robot_aabb = _collect_aabb(_robot, AABB())
	var center := _robot_aabb.position + _robot_aabb.size * 0.5
	var radius := _robot_aabb.size.length() * 0.5
	if radius <= 0.0:
		radius = 0.3
	_cam_target = center
	_cam_dist = radius * 2.5
	_cam_min_dist = radius * 0.15
	_update_camera()


func _build_ground() -> void:
	var center := _robot_aabb.position + _robot_aabb.size * 0.5
	var radius := _robot_aabb.size.length() * 0.5
	if radius <= 0.0:
		radius = 0.3
	var mi := MeshInstance3D.new()
	mi.name = "Ground"
	var plane := PlaneMesh.new()
	var size := maxf(radius * 6.0, 1.0)
	plane.size = Vector2(size, size)
	mi.mesh = plane
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.24, 0.26, 0.29)
	mat.roughness = 0.9
	mi.set_surface_override_material(0, mat)
	mi.position = Vector3(center.x, _robot_aabb.position.y, center.z)
	add_child(mi)


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
