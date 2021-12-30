extends Node
class_name DebugVector


@export var start: Vector3:
	set(value):
		start = value
		_update_placement()

@export var direction: Vector3:
	set(value):
		direction = value
		_update_placement()

@export var thickness := 0.02:
	set(value):
		thickness = value
		_update_geometry()

@export var tip_length := 0.04:
	set(value):
		tip_length = value
		_update_geometry()

var _shaft: MeshInstance3D = null
var _tip: MeshInstance3D = null


func _update_geometry():
	if is_inside_tree():
		_shaft.mesh.top_radius = thickness * 0.5
		_shaft.mesh.bottom_radius = thickness * 0.5
		_shaft.mesh.height = 1.0

		_tip.mesh.top_radius = 0.0
		_tip.mesh.bottom_radius = thickness
		_tip.mesh.height = 1.0


func _update_placement():
	if is_inside_tree():
		assert(_shaft)
		assert(_tip)
		
		var height := maxf(direction.length() - tip_length, 0.0)
		var y: Vector3 = direction.normalized()
		var x := y.cross(Vector3(0, 1, 0)).normalized()
		if x.length_squared() < 1.0e-6:
			x = y.cross(Vector3(1, 0, 0)).normalized()
		var z := x.cross(y)
		_shaft.transform = Transform3D(x, y * height, z, start + y * height * 0.5)
		_tip.transform = Transform3D(x, y * tip_length, z, start + y * height)


func _enter_tree():
	_shaft = MeshInstance3D.new()
	_shaft.mesh = CylinderMesh.new()
	_tip = MeshInstance3D.new()
	_tip.mesh = CylinderMesh.new()
	_update_geometry()
	_update_placement()

	add_child(_shaft)
	add_child(_tip)


func _exit_tree():
	remove_child(_shaft)
	remove_child(_tip)
	_shaft = null
	_tip = null
