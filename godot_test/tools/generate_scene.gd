extends SceneTree
## Headless tool: robot.json + meshes -> a real, saved RobotScene.tscn with
## a camera, ground plane and light around it — a real test environment,
## not just the bare robot tree.
##
## This computes zero kinematics. It only calls the same CadRobotLoader the
## runtime main.gd uses (which now also applies the CAD Z-up -> Godot Y-up
## conversion — see robot_loader.gd's _apply_up_conversion), measures the
## resulting mesh geometry to fit a camera/ground plane, and saves the
## result to disk.
##
## Run via: godot --headless --script res://tools/generate_scene.gd

const LOADER := preload("res://addons/cad_robot_importer/robot_loader.gd")
const ROBOT_JSON := "res://robot_data/robot.json"
const OVERLAY_JSON := "res://robot_data/debug_overlay.json"
const OUT_SCENE := "res://RobotScene.tscn"

const CAM_YAW := -0.7
const CAM_PITCH := 0.5
const CAM_DIST_MULT := 2.5


func _initialize() -> void:
	var robot := LOADER.load_robot(ROBOT_JSON)
	if robot == null:
		push_error("generate_scene: failed to build robot tree from %s" % ROBOT_JSON)
		quit(1)
		return

	var aabb := _collect_aabb(robot, Transform3D.IDENTITY, AABB())
	var has_geometry := aabb.size != Vector3.ZERO
	var center := aabb.position + aabb.size * 0.5 if has_geometry else Vector3.ZERO
	var radius := aabb.size.length() * 0.5 if has_geometry else 0.3
	if radius <= 0.0:
		radius = 0.3
	var bottom_y := aabb.position.y if has_geometry else 0.0

	robot.add_child(_build_camera(center, radius))
	robot.add_child(_build_ground(center, bottom_y, radius))
	robot.add_child(_build_light(center, radius))

	# Joint nodes are bare, mesh-less Node3D — nothing to click on in the 3D
	# viewport. Drop a visible, clickable sphere at each pivot (from the
	# pipeline's own debug_overlay.json) so every joint has a visual anchor,
	# not just a Scene-dock row.
	var robot_root: Node3D = robot.get_node("RobotRoot")
	if robot_root != null:
		LOADER.build_pivot_markers(robot_root, OVERLAY_JSON)

	# Attach the same interactive controller main.tscn uses (per-joint
	# control, Test Motion, tint, orbit camera). It has no @tool
	# annotation, so it only runs in Play mode — the Editor view stays the
	# plain, static, saved scene. Its _ready() detects the "RobotRoot"
	# child already here and uses this exact tree as-is instead of
	# rebuilding, so Editor edits saved into this file are what Play shows.
	robot.set_script(preload("res://main.gd"))

	_set_owner_recursive(robot, robot)

	var packed := PackedScene.new()
	var err := packed.pack(robot)
	if err != OK:
		push_error("generate_scene: pack() failed: %s" % err)
		quit(1)
		return

	err = ResourceSaver.save(packed, OUT_SCENE)
	if err != OK:
		push_error("generate_scene: save failed: %s" % err)
		quit(1)
		return

	print("Saved ", OUT_SCENE, " (", _count_nodes(robot), " nodes, radius=", "%.2f" % radius, "m)")
	quit(0)


func _build_camera(center: Vector3, radius: float) -> Camera3D:
	var cam := Camera3D.new()
	cam.name = "Camera3D"
	cam.current = true
	var offset := Vector3(
		cos(CAM_PITCH) * sin(CAM_YAW),
		sin(CAM_PITCH),
		cos(CAM_PITCH) * cos(CAM_YAW)
	) * (radius * CAM_DIST_MULT)
	var eye := center + offset
	cam.transform = _look_at_transform(eye, center)
	return cam


## Camera3D.look_at() requires being inside the tree; build the transform by
## hand instead (same math, works headless on an unparented node).
func _look_at_transform(eye: Vector3, target: Vector3) -> Transform3D:
	var fwd := (target - eye).normalized()
	var up := Vector3.UP
	if abs(fwd.dot(up)) > 0.999:
		up = Vector3.RIGHT
	var right := fwd.cross(up).normalized()
	var true_up := right.cross(fwd).normalized()
	# Camera looks down -Z by convention.
	var b := Basis(right, true_up, -fwd)
	return Transform3D(b, eye)


func _build_ground(center: Vector3, bottom_y: float, radius: float) -> MeshInstance3D:
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
	mi.position = Vector3(center.x, bottom_y, center.z)
	return mi


func _build_light(center: Vector3, radius: float) -> DirectionalLight3D:
	var light := DirectionalLight3D.new()
	light.name = "DirectionalLight3D"
	light.shadow_enabled = true
	var eye := center + Vector3(radius, radius * 1.6, radius)
	light.transform = _look_at_transform(eye, center)
	return light


func _collect_aabb(node: Node, world: Transform3D, acc: AABB) -> AABB:
	var t := world
	if node is Node3D:
		t = world * (node as Node3D).transform
	if node is MeshInstance3D and node.mesh != null:
		var world_aabb: AABB = t * node.mesh.get_aabb()
		acc = world_aabb if acc.size == Vector3.ZERO and acc.position == Vector3.ZERO else acc.merge(world_aabb)
	for c in node.get_children():
		acc = _collect_aabb(c, t, acc)
	return acc


func _set_owner_recursive(node: Node, owner: Node) -> void:
	for c in node.get_children():
		c.owner = owner
		if c.scene_file_path == "":
			_set_owner_recursive(c, owner)


func _count_nodes(node: Node) -> int:
	var n := 1
	for c in node.get_children():
		n += _count_nodes(c)
	return n
