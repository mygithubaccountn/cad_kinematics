extends RefCounted
class_name CadRobotLoader
## Consumes pipeline robot.json. No pivot/axis computation — apply only.

static func load_robot(robot_json_path: String) -> Node3D:
	var file := FileAccess.open(robot_json_path, FileAccess.READ)
	if file == null:
		push_error("Cannot open %s" % robot_json_path)
		return null
	var data = JSON.parse_string(file.get_as_text())
	if typeof(data) != TYPE_DICTIONARY:
		push_error("Invalid robot.json")
		return null
	var base_dir := robot_json_path.get_base_dir()
	return build_tree(data, base_dir)


static func build_tree(data: Dictionary, base_dir: String) -> Node3D:
	var root := Node3D.new()
	root.name = str(data.get("name", "Robot"))

	var links: Array = data.get("links", [])
	var joints: Array = data.get("joints", [])
	var base_id := str(data.get("base_link", ""))

	var nodes: Dictionary = {}  # link_id -> Node3D
	for link in links:
		var n := Node3D.new()
		n.name = str(link.get("name", link.get("id", "link")))
		var mesh_rel := str(link.get("mesh", ""))
		if mesh_rel != "":
			var mesh_path := base_dir.path_join(mesh_rel)
			if ResourceLoader.exists(mesh_path):
				_attach_mesh(n, mesh_path)
			else:
				push_warning("Missing mesh: %s" % mesh_path)
		nodes[str(link["id"])] = n

	if not nodes.has(base_id):
		# Phase 0: no kinematic base — attach everything under root
		if joints.is_empty():
			for link_id in nodes.keys():
				root.add_child(nodes[link_id])
			root.set_meta("cad_robot", true)
			root.set_meta("phase", 0)
			_apply_up_conversion(root, data)
			return root
		push_error("base_link missing: %s" % base_id)
		_apply_up_conversion(root, data)
		return root

	root.add_child(nodes[base_id])

	# Phase 0: empty joints → flat display (all other links under root)
	if joints.is_empty():
		for link_id in nodes.keys():
			if link_id == base_id:
				continue
			root.add_child(nodes[link_id])
		root.set_meta("cad_robot", true)
		root.set_meta("phase", 0)
		root.set_meta("frame", str(data.get("frame", "cad_z_up")))
		_apply_up_conversion(root, data)
		return root

	# Parent joints: child node placed at origin under parent
	var by_child: Dictionary = {}
	for j in joints:
		by_child[str(j["child"])] = j

	# Attach in any order; ensure parent exists
	var pending: Array = joints.duplicate()
	var guard := 0
	while pending.size() > 0 and guard < 10000:
		guard += 1
		var j: Dictionary = pending.pop_front()
		var parent_id := str(j["parent"])
		var child_id := str(j["child"])
		if not nodes.has(parent_id) or not nodes.has(child_id):
			continue
		var parent_n: Node3D = nodes[parent_id]
		var child_n: Node3D = nodes[child_id]
		if child_n.get_parent() != null:
			continue
		if parent_n.get_parent() == null and parent_id != base_id:
			pending.append(j)
			continue
		parent_n.add_child(child_n)
		var origin: Array = j.get("origin", [0, 0, 0])
		child_n.position = Vector3(origin[0], origin[1], origin[2])
		# Store axis for runtime drivers (main.gd's set_joint reads these)
		var axis: Array = j.get("axis", [0, 0, 1])
		var jtype := str(j.get("type", "revolute"))
		var jid := str(j.get("id", ""))
		child_n.set_meta("joint_id", jid)
		child_n.set_meta("joint_type", jtype)
		child_n.set_meta("joint_axis", Vector3(axis[0], axis[1], axis[2]))
		child_n.set_meta("joint_angle", 0.0)
		_attach_joint_data(child_n, j, jid, jtype, origin, axis)

	# Attach any remaining orphans under root
	for link_id in nodes.keys():
		var n: Node3D = nodes[link_id]
		if n.get_parent() == null:
			root.add_child(n)

	root.set_meta("cad_robot", true)
	root.set_meta("frame", str(data.get("frame", "gltf_y_up")))
	_apply_up_conversion(root, data)
	return root


## robot.json is currently always written with frame="cad_z_up" (the
## pipeline's to_gltf_y_up export flag exists — src/common/frames.py,
## src/exporter/scene.py — but isn't enabled on the CLI path). Rather than
## touch the pipeline, apply that exact same, already-defined conversion
## here as the one place robot.json meets Godot's Y-up engine: rigidly
## rotate the whole assembled tree, so mesh geometry and joint pivots move
## together and every relative pivot/axis stays exactly as computed.
## Matches common/frames.py::cad_z_up_to_gltf_y_up(): (x,y,z) -> (x,z,-y).
static func _apply_up_conversion(root: Node3D, data: Dictionary) -> void:
	if str(data.get("frame", "cad_z_up")) != "cad_z_up":
		return  # already Y-up (pipeline flag enabled) — nothing to do
	var b := Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))
	root.transform = Transform3D(b, Vector3.ZERO) * root.transform


## GLB import produces a PackedScene (a Node3D/MeshInstance3D subtree), not
## a bare Mesh. Instantiate it, pull out the actual MeshInstance3D (and its
## mesh + any per-surface material overrides) and re-parent that directly
## under the link so the Scene dock shows a real MeshInstance3D per link,
## not an opaque scene-instance node one has to expand to find. Geometry
## isn't touched — same Mesh resource, just adopted as a direct child.
static func _attach_mesh(parent: Node3D, mesh_path: String) -> void:
	var res: Resource = load(mesh_path)
	if res == null:
		push_warning("Failed to load mesh resource: %s" % mesh_path)
		return
	if res is PackedScene:
		var inst := (res as PackedScene).instantiate()
		var found := _find_mesh_instance(inst)
		if found != null:
			var mi := MeshInstance3D.new()
			mi.name = "Mesh"
			mi.mesh = _externalize_mesh(found.mesh, mesh_path)
			mi.transform = _local_transform_relative_to(found, inst)  # relative to the glb root, normally identity
			for si in range(found.get_surface_override_material_count()):
				var mat := found.get_surface_override_material(si)
				if mat != null:
					mi.set_surface_override_material(si, mat)
			parent.add_child(mi)
			inst.free()
		else:
			push_warning("No MeshInstance3D found inside %s — attaching raw instance" % mesh_path)
			inst.name = "Mesh"
			parent.add_child(inst)
	elif res is Mesh:
		var mi := MeshInstance3D.new()
		mi.name = "Mesh"
		mi.mesh = res
		parent.add_child(mi)
	else:
		push_warning("Unexpected mesh resource type for %s: %s" % [mesh_path, res])


## Compose local (never global_transform — the instantiated glb subtree
## isn't necessarily inside a live SceneTree yet, so global_transform can't
## be trusted) transforms from `node` up to `ancestor`.
static func _local_transform_relative_to(node: Node3D, ancestor: Node) -> Transform3D:
	var t := Transform3D.IDENTITY
	var cur: Node3D = node
	while cur != null and cur != ancestor:
		t = cur.transform * t
		cur = cur.get_parent() as Node3D
	return t


## The mesh living inside an imported .glb's PackedScene is an anonymous
## sub-resource of that scene (path like ".../link.glb::ArrayMesh_xxx") —
## when a *different* scene (RobotScene.tscn) references it directly,
## Godot's packer can't point to it externally and silently embeds the
## full geometry inline instead (a single mesh can add hundreds of KB of
## base64 to the .tscn text). Save it once as its own standalone .tres next
## to the source .glb so the packer keeps referencing it externally instead.
static func _externalize_mesh(mesh: Mesh, source_glb_path: String) -> Mesh:
	if mesh == null:
		return null
	var tres_path := source_glb_path.get_basename() + ".mesh.tres"
	if ResourceLoader.exists(tres_path):
		var cached: Resource = load(tres_path)
		if cached is Mesh:
			return cached
	var err := ResourceSaver.save(mesh, tres_path)
	if err != OK:
		push_warning("Could not externalize mesh to %s (err=%s) — embedding inline" % [tres_path, err])
		return mesh
	return load(tres_path)


static func _find_mesh_instance(node: Node) -> MeshInstance3D:
	if node is MeshInstance3D:
		return node
	for c in node.get_children():
		var found := _find_mesh_instance(c)
		if found != null:
			return found
	return null


## Editor-visible mirror of the joint fields already on the node's meta —
## same values, but as a typed Resource so they show as real Inspector
## fields (Transform/Position is still the thing that actually drives the
## pivot; this is read-only bookkeeping for a human looking at the scene).
static func _attach_joint_data(node: Node3D, j: Dictionary, jid: String, jtype: String, origin: Array, axis: Array) -> void:
	node.set_script(preload("res://addons/cad_robot_importer/robot_link.gd"))
	var jd := JointData.new()
	jd.joint_id = jid
	jd.joint_type = jtype
	jd.parent_link = str(j.get("parent", ""))
	jd.child_link = str(j.get("child", ""))
	jd.pivot = Vector3(origin[0], origin[1], origin[2])
	jd.axis = Vector3(axis[0], axis[1], axis[2])
	jd.confidence = float(j.get("confidence", 1.0))
	node.joint = jd


## Rotate / translate a joint child node. Pipeline already set pivot as node origin.
static func set_joint(node: Node3D, value: float) -> void:
	if not node.has_meta("joint_type"):
		return
	var jtype := str(node.get_meta("joint_type"))
	var axis: Vector3 = node.get_meta("joint_axis")
	if jtype == "revolute":
		node.rotation = Vector3.ZERO
		node.rotate_object_local(axis.normalized(), value)
		node.set_meta("joint_angle", value)
	elif jtype == "prismatic":
		# Rest local position stored? Use meta rest_position if set
		var rest: Vector3 = node.get_meta("rest_position") if node.has_meta("rest_position") else node.position
		if not node.has_meta("rest_position"):
			node.set_meta("rest_position", node.position)
			rest = node.position
		node.position = rest + axis.normalized() * value
