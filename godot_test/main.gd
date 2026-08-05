extends Node3D
## Load robot.json, draw pivot/axis debug, wiggle joints (CadRobotLoader contract).

const ROBOT_JSON := "res://robot_data/robot.json"
const OVERLAY_JSON := "res://robot_data/debug_overlay.json"
const LOADER_SCRIPT := preload("res://addons/cad_robot_importer/robot_loader.gd")

var _robot: Node3D
var _joint_nodes: Array[Node3D] = []
var _debug: Node3D
var _t := 0.0


func _ready() -> void:
	_robot = LOADER_SCRIPT.load_robot(ROBOT_JSON)
	if _robot == null:
		push_error("Robot yüklenemedi: %s" % ROBOT_JSON)
		return
	add_child(_robot)
	_collect_joints(_robot)
	_tint_links()
	_build_debug_overlay()
	print("Robot yüklendi. Hareketli joint: ", _joint_nodes.size())
	_fit_camera()
	# One-shot contract self-check (also covered by Python godot_runtime_report)
	_runtime_smoke()


func _process(delta: float) -> void:
	_t += delta
	var angle := sin(_t) * 0.4
	for n in _joint_nodes:
		LOADER_SCRIPT.set_joint(n, angle)


func _collect_joints(node: Node) -> void:
	if node.has_meta("joint_type"):
		var t := str(node.get_meta("joint_type"))
		if t == "revolute" or t == "prismatic":
			_joint_nodes.append(node as Node3D)
	for c in node.get_children():
		_collect_joints(c)


func _tint_links() -> void:
	if not FileAccess.file_exists(OVERLAY_JSON):
		return
	var f := FileAccess.open(OVERLAY_JSON, FileAccess.READ)
	if f == null:
		return
	var data = JSON.parse_string(f.get_as_text())
	if typeof(data) != TYPE_DICTIONARY:
		return
	var colors: Dictionary = data.get("link_colors", {})
	_apply_tint(_robot, colors)


func _apply_tint(node: Node, colors: Dictionary) -> void:
	# Link Node3D name == link id in our export
	if node is Node3D and colors.has(node.name):
		var rgb: Array = colors[node.name].get("rgb", [0.8, 0.8, 0.8])
		var col := Color(float(rgb[0]), float(rgb[1]), float(rgb[2]))
		for c in node.get_children():
			if c is MeshInstance3D:
				var mi := c as MeshInstance3D
				if mi.mesh == null:
					continue
				for si in range(mi.mesh.get_surface_count()):
					var mat := StandardMaterial3D.new()
					mat.albedo_color = col
					mi.set_surface_override_material(si, mat)
	for c in node.get_children():
		_apply_tint(c, colors)


func _build_debug_overlay() -> void:
	_debug = Node3D.new()
	_debug.name = "DebugOverlay"
	add_child(_debug)
	if not FileAccess.file_exists(OVERLAY_JSON):
		push_warning("debug_overlay.json yok — pipeline validate çalıştır")
		return
	var f := FileAccess.open(OVERLAY_JSON, FileAccess.READ)
	var data = JSON.parse_string(f.get_as_text())
	if typeof(data) != TYPE_DICTIONARY:
		return
	for m in data.get("markers", []):
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
	for ax in data.get("axes", []):
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
	var cam := $Camera3D as Camera3D
	if cam == null or _robot == null:
		return
	cam.look_at(Vector3(0.2, 0.2, 0.0), Vector3.UP)
