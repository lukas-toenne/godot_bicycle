extends Node
class_name DebugVector


export(Vector3) var start setget _set_start
export(Vector3) var direction setget _set_direction
export(float) var thickness := 0.02 setget _set_thickness
export(float) var tip_length := 0.04

var _shaft: MeshInstance = null
var _tip: MeshInstance = null


func _update_geometry():
	if is_inside_tree():
		_shaft.mesh.top_radius = thickness * 0.5
		_shaft.mesh.bottom_radius = thickness * 0.5
		_shaft.mesh.height = 1.0

		_tip.mesh.top_radius = 0.0
		_tip.mesh.bottom_radius = thickness
		_tip.mesh.height = 1.0


func _set_thickness(value):
	thickness = value
	_update_geometry()


func _set_tip_length(value):
	tip_length = value
	_update_geometry()


func _update_placement():
	if is_inside_tree():
		assert(_shaft)
		assert(_tip)
		
		var height := max(direction.length() - tip_length, 0.0)
		var y: Vector3 = direction.normalized()
		var x := y.cross(Vector3(0, 1, 0)).normalized()
		if x.length_squared() < 1.0e-6:
			x = y.cross(Vector3(1, 0, 0)).normalized()
		var z := x.cross(y)
		_shaft.transform = Transform(x, y * height, z, start + y * height * 0.5)
		_tip.transform = Transform(x, y * tip_length, z, start + y * height)


func _set_start(value):
	start = value
	_update_placement()


func _set_direction(value):
	direction = value
	_update_placement()


func _enter_tree():
	_shaft = MeshInstance.new()
	_shaft.mesh = CylinderMesh.new()
	_tip = MeshInstance.new()
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
