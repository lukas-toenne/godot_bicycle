extends Node
class_name VehicleController

var vehicle: Bicycle setget ,_get_vehicle

export(float, -1.0, 1.0) var forward_movement = 0.0
export(float) var side_movement = 0.0
export(float) var turn = 0.0
export(float) var brake = 0.0
export(bool) var boost = false


var forward_speed setget ,_get_forward_speed
var forward_speed_fraction setget ,_get_forward_speed_fraction
var forward_speed_range setget ,_get_forward_speed_range

# TODO lots more could be added here for AI navigation purposes


func enter_vehicle(value):
	if vehicle != null:
		# Have to exit current vehicle first.
		return false
	if value.controller != null:
		# Target vehicle has another controller.
		return false
	vehicle = value
	vehicle.controller = self
	return true


func exit_vehicle():
	if vehicle == null:
		# Can only exit if currently controlling a vehicle.
		return false
	assert(vehicle.controller == self)
	vehicle.controller = null
	vehicle = null
	return true


func _get_vehicle():
	return vehicle


func _get_forward_speed():
	return 0.0


func _get_forward_speed_fraction():
	var speed = _get_forward_speed()
	var speed_range = _get_forward_speed_range()
	if speed >= 0.0:
		return speed / speed_range[1] if speed_range[1] > 0.0 else 0.0
	else:
		return speed / speed_range[0] if speed_range[0] < 0.0 else 0.0


func _get_forward_speed_range():
	return [-1.0, 1.0]
