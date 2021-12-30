extends RigidDynamicBody3D

@export var lifetime := 1.0

var _time := 0.0


func _ready():
	_time = lifetime

func _process(delta):
	_time -= delta
	if _time <= 0.0:
		queue_free()
		set_process(false)
