@tool
extends EditorPlugin

const IMPORTER := preload("res://addons/cad_robot_importer/robot_loader.gd")

func _enter_tree() -> void:
	add_tool_menu_item("Import CAD Robot (robot.json)", _on_import)

func _exit_tree() -> void:
	remove_tool_menu_item("Import CAD Robot (robot.json)")

func _on_import() -> void:
	var dialog := EditorFileDialog.new()
	dialog.file_mode = EditorFileDialog.FILE_MODE_OPEN_FILE
	dialog.add_filter("*.json ; Robot Description")
	dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
	get_editor_interface().get_base_control().add_child(dialog)
	dialog.popup_centered_ratio(0.6)
	dialog.file_selected.connect(func(path: String) -> void:
		var root := IMPORTER.load_robot(path)
		if root == null:
			push_error("CAD Robot Importer: failed to load %s" % path)
			dialog.queue_free()
			return
		var edited := get_editor_interface().get_edited_scene_root()
		if edited:
			edited.add_child(root)
			root.owner = edited
			_set_owner_recursive(root, edited)
		else:
			push_warning("Open a scene first; placing robot as orphan node.")
		dialog.queue_free()
	)

func _set_owner_recursive(node: Node, owner: Node) -> void:
	for c in node.get_children():
		c.owner = owner
		# Don't recurse into an instanced sub-scene's internals (e.g. a GLB
		# mesh) — its own nodes belong to that sub-scene, not this one;
		# only the instance root itself needs an owner here.
		if c.scene_file_path == "":
			_set_owner_recursive(c, owner)
