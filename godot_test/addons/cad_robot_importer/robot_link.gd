extends Node3D
class_name RobotLink
## Attached only to link nodes that are the *child* side of a joint (i.e.
## nodes whose local Transform origin is a pipeline-resolved pivot). Plain
## links (base, or links without an inbound joint) stay plain Node3D.

@export var joint: JointData
