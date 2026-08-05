extends SceneTree
## Headless tool: robot.json + meshes -> a real, saved RobotScene.tscn.
##
## This is the only thing that turns pipeline output into something the
## Godot Editor can show in the Scene dock / Inspector. It does not compute
## any kinematics — it just calls the same CadRobotLoader the runtime
## main.gd uses, then persists the resulting node tree to disk.
##
## Run via: godot --headless --script res://tools/generate_scene.gd

const LOADER := preload("res://addons/cad_robot_importer/robot_loader.gd")
const ROBOT_JSON := "res://robot_data/robot.json"
const OUT_SCENE := "res://RobotScene.tscn"


func _initialize() -> void:
	var root := LOADER.load_robot(ROBOT_JSON)
	if root == null:
		push_error("generate_scene: failed to build robot tree from %s" % ROBOT_JSON)
		quit(1)
		return

	_set_owner_recursive(root, root)

	var packed := PackedScene.new()
	var err := packed.pack(root)
	if err != OK:
		push_error("generate_scene: pack() failed: %s" % err)
		quit(1)
		return

	err = ResourceSaver.save(packed, OUT_SCENE)
	if err != OK:
		push_error("generate_scene: save failed: %s" % err)
		quit(1)
		return

	print("Saved ", OUT_SCENE, " (", _count_nodes(root), " nodes)")
	quit(0)


func _set_owner_recursive(node: Node, owner: Node) -> void:
	for c in node.get_children():
		c.owner = owner
		# Don't recurse into an instanced sub-scene's internals (a GLB mesh)
		# — those nodes belong to that sub-scene, not the new one.
		if c.scene_file_path == "":
			_set_owner_recursive(c, owner)


func _count_nodes(node: Node) -> int:
	var n := 1
	for c in node.get_children():
		n += _count_nodes(c)
	return n
