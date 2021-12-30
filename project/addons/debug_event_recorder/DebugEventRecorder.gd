extends Node


var _events = Dictionary()

var _vector_shaft: Mesh = null
var _vector_tip: Mesh = null
var _spring_shaft: Mesh = null
var _spring_piston: Mesh = null

@export var vector_thickness := 0.02
@export var vector_tip_length := 0.04
@export var spring_thickness := 0.02
@export var spring_piston_height := 0.02
@export var spring_piston_radius := 0.08


func _enter_tree():
	_vector_shaft = CylinderMesh.new()
	_vector_shaft.top_radius = 0.5
	_vector_shaft.bottom_radius = 0.5
	_vector_shaft.height = 1.0

	_vector_tip = CylinderMesh.new()
	_vector_tip.top_radius = 0.0
	_vector_tip.bottom_radius = 1.0
	_vector_tip.height = 1.0

	_spring_shaft = CylinderMesh.new()
	_spring_shaft.top_radius = 1.0
	_spring_shaft.bottom_radius = 1.0
	_spring_shaft.height = 1.0

	_spring_piston = CylinderMesh.new()
	_spring_piston.top_radius = 1.0
	_spring_piston.bottom_radius = 1.0
	_spring_piston.height = 1.0


func _exit_tree():
	_vector_shaft = null
	_vector_tip = null
	_spring_shaft = null
	_spring_piston = null


func _get_or_create_event(node: Node, key, on_create: Callable):
	var node_items = _events.get(node, null)
	if node_items == null:
		node_items = Dictionary()
		_events[node] = node_items
	
	var item = node_items.get(key, null)
	if item == null:
		item = on_create.call()
		node_items[key] = item
	return item


func clear_event(node: Node, key):
	var node_items = _events.get(node, null)
	if node_items != null:
		var item = node_items.get(key, null)
		if item != null:
			for meshinst in item:
				remove_child(meshinst)
				meshinst.queue_free()
			node_items.erase(key)
		if node_items.is_empty():
			_events.erase(node)


func _make_vector():
	var shaft = MeshInstance3D.new()
	shaft.mesh = _vector_shaft
	add_child(shaft)
	var tip = MeshInstance3D.new()
	tip.mesh = _vector_tip
	add_child(tip)
	return [shaft, tip]


func record_vector(node: Node, key, start: Vector3, direction: Vector3):
	var vec = _get_or_create_event(node, key, self._make_vector)

	var y: Vector3 = direction.normalized()
	var x := y.cross(Vector3(0, 1, 0)).normalized()
	if x.length_squared() < 1.0e-6:
		x = y.cross(Vector3(1, 0, 0)).normalized()
	var z := x.cross(y)
	var height := maxf(direction.length() - vector_tip_length, 0.0)
	vec[0].transform = Transform3D(x * vector_thickness, y * height,            z * vector_thickness, start + y * height * 0.5)
	vec[1].transform = Transform3D(x * vector_thickness, y * vector_tip_length, z * vector_thickness, start + y * height)


func _make_spring():
	var shaft = MeshInstance3D.new()
	shaft.mesh = _spring_shaft
	add_child(shaft)
	var piston = MeshInstance3D.new()
	piston.mesh = _spring_piston
	add_child(piston)
	return [shaft, piston]


func record_spring(node: Node, key, origin: Vector3, direction: Vector3, rest_length: float, length: float):
	var spring = _get_or_create_event(node, key, self._make_spring)

	var y: Vector3 = direction.normalized()
	var x := y.cross(Vector3(0, 1, 0)).normalized()
	if x.length_squared() < 1.0e-6:
		x = y.cross(Vector3(1, 0, 0)).normalized()
	var z := x.cross(y)
	spring[0].transform = Transform3D(x * spring_thickness,     y * rest_length,          z * spring_thickness,     origin + y * rest_length * 0.5)
	spring[1].transform = Transform3D(x * spring_piston_radius, y * spring_piston_height, z * spring_piston_radius, origin + y * (length - 0.5 * spring_piston_height))


#func _process(delta):
#	_events.clear()
