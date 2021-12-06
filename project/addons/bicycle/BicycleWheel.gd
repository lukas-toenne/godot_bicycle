extends Spatial
class_name BicycleWheel


class RaycastInfo:
	# Contact vectors in world space
	var hard_point: Vector3
	var wheel_direction: Vector3
	var wheel_axle: Vector3
	var contact_normal: Vector3
	var contact_point: Vector3
	var relative_contact_velocity: Vector3
	
	var inv_ground_mass: float
	var inv_ground_inertia: Basis
	var ground_linear_velocity: Vector3
	var ground_angular_velocity: Vector3
	
	var suspension_relative_velocity: float
	var clipped_inv_contact_dot_suspension: float

	var suspension_length: float
	var is_in_contact: bool
	var ground_object: CollisionObject


export(float) var torque := 0.0
export(float) var steering_angle := 0.0

export(bool) var use_as_traction := false
export(bool) var use_as_steering := false

export(float) var roll_influence := 0.1
export(float) var radius := 0.5;
export(float) var rest_length := 0.15
export(float) var friction_slip := 10.5

export(float) var travel := 0.05
export(float) var stiffness := 5.88
export(float) var max_force := 6000.0

export(float) var compression_damping := 0.83
export(float) var relaxation_damping := 0.88


#var _engine_force := 0.0
#var _brake := 0.0
#var _steering := 0.0
#
#var _use_as_traction := false
#var _use_as_steering := false
#
#var _roll_influence := 0.1
#var _radius := 0.5;
#var _rest_length := 0.15
#var _friction_slip := 10.5
#
#var _travel := 0.05
#var _stiffness := 5.88
#var _max_force := 6000
#
#var _compression_damping := 0.83
#var _relaxation_damping := 0.88

# Rest transform of the wheel in parent space.
var _rest_transform := Transform.IDENTITY

# Raycast result of the last physics step
var _raycast_info := RaycastInfo.new()


#func _get_property_list():
#	var props = [
#		{
#			name = "Bicycle Wheel",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_CATEGORY,
#		},
#		{
#			name = "Per-Wheel Motion",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_GROUP,
#		},
#		{
#			name = "engine_force",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#			hint = PROPERTY_HINT_RANGE,
#			hint_string = "-1024,1024",
#		},
#		{
#			name = "brake",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "steering",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#
#		{
#			name = "Vehicle Body Motion",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_GROUP,
#		},
#		{
#			name = "use_as_traction",
#			type = TYPE_BOOL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "use_as_steering",
#			type = TYPE_BOOL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#
#		{
#			name = "Wheel",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_GROUP,
#		},
#		{
#			name = "roll_influence",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "radius",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "rest_length",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "friction_slip",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#
#
#		{
#			name = "Suspension",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_GROUP,
#		},
#		{
#			name = "travel",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "stiffness",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "max_force",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#
#		{
#			name = "Damping",
#			type = TYPE_NIL,
#			usage = PROPERTY_USAGE_GROUP,
#		},
#		{
#			name = "compression",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#		{
#			name = "relaxation",
#			type = TYPE_REAL,
#			usage = PROPERTY_USAGE_DEFAULT,
#		},
#	]
#	return props
#
#
#func _get(key: String):
#	if key == "engine_force":
#		return _engine_force
#	if key == "brake":
#		return _brake
#	if key == "steering":
#		return _steering
#
#	if key == "use_as_traction":
#		return _use_as_traction
#	if key == "use_as_steering":
#		return _use_as_steering
#
#	if key == "roll_influence":
#		return _roll_influence
#	if key == "radius":
#		return _radius
#	if key == "rest_length":
#		return _rest_length
#	if key == "friction_slip":
#		return _friction_slip
#
#	if key == "travel":
#		return _travel
#	if key == "stiffness":
#		return _stiffness
#	if key == "max_force":
#		return _max_force
#
#	if key == "compression":
#		return _compression_damping
#	if key == "relaxation":
#		return _relaxation_damping
#
#
#func _set(key: String, value):
#	if key == "engine_force":
#		_engine_force = value
#	if key == "brake":
#		_brake = value
#	if key == "steering":
#		_steering = value
#
#	if key == "use_as_traction":
#		_use_as_traction = value
#	if key == "use_as_steering":
#		_use_as_steering = value
#
#	if key == "roll_influence":
#		_roll_influence = value
#	if key == "radius":
#		_radius = value
#	if key == "rest_length":
#		_rest_length = value
#	if key == "friction_slip":
#		_friction_slip = value
#
#	if key == "travel":
#		_travel = value
#	if key == "stiffness":
#		_stiffness = value
#	if key == "max_force":
#		_max_force = value
#
#	if key == "compression":
#		_compression_damping = value
#	if key == "relaxation":
#		_relaxation_damping = value


func _notification(what):
	# XXX Should check for vehicle class of the parent node instead of has_method,
	# but this creates a circular dependency.
	# Might be better to do this on the vehicle side.
	if what == Node.NOTIFICATION_ENTER_TREE:
		var bike = get_parent()
		if bike.has_method("add_wheel"):
			bike.add_wheel(self)
			_update_chassis_relation()
	if what == Node.NOTIFICATION_EXIT_TREE:
		var bike = get_parent()
		if bike.has_method("remove_wheel"):
			bike.remove_wheel(self)


func _update_chassis_relation():
	_rest_transform = transform
