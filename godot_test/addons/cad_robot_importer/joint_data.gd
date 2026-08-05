extends Resource
class_name JointData
## Read-only mirror of one robot.json joint entry, attached to its child
## Link node so pivot/axis/type/confidence show up as real typed fields in
## the Godot Inspector instead of being buried in generic node metadata.
## Pipeline-authored — the pivot that matters for kinematics is the node's
## own Transform (position); this resource is for inspection, not driving.

@export var joint_id: String = ""
@export var joint_type: String = "revolute"
@export var parent_link: String = ""
@export var child_link: String = ""
@export var pivot: Vector3 = Vector3.ZERO
@export var axis: Vector3 = Vector3.UP
@export var confidence: float = 1.0
