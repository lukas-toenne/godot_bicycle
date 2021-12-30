extends Node
class_name VehicleController

var vehicle: Bicycle:
	get:
		return vehicle

# XXX bug: negative values don't work in export_range
# https://github.com/godotengine/godot/issues/41183
#@export_range(-1.0, 1.0) var forward_movement := 0.0
@export var forward_movement := 0.0
@export var side_movement := 0.0
@export var turn := 0.0
@export var brake := 0.0
@export var boost := false

var forward_speed: float:
	get:
		return 0.0

var forward_speed_fraction: float:
	get:
		var speed = forward_speed
		var speed_range = forward_speed_range
		if speed >= 0.0:
			return speed / speed_range[1] if speed_range[1] > 0.0 else 0.0
		else:
			return speed / speed_range[0] if speed_range[0] < 0.0 else 0.0

var forward_speed_range: Array[float]:
	get:
		return [-1.0, 1.0]

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
