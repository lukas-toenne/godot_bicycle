extends Bicycle

# Based on vehicle demo by Bastiaan Olij
# https://github.com/BastiaanOlij/vehicle-demo
# Released under MIT license
#
# With a couple of additions to simulate the clutch.

export(float) var MAX_ENGINE_TORQUE = 700.0
export(float) var MAX_ENGINE_RPM = 8000.0
export(Curve) var ENGINE_TORQUE_CURVE = preload("./DefaultTorqueCurve.tres")
export(Array) var GEAR_RATIOS = [ -2.5, 0.0, 2.69, 2.01, 1.59, 1.32, 1.13, 1.0 ] 
export(float) var FINAL_DRIVE_RATIO = 3.38
# Approximated moments of inertial as cylinders of mass M and radius R: I = (M * R^2) / 2
export(float) var WHEEL_INERTIA = 100.0 * 0.4*0.4 / 2.0
export(float) var ENGINE_INERTIA = 300.0 * 0.1*0.1 / 2.0

export(float) var MAX_BRAKE_TORQUE = 50.0

export(float) var MAX_STEER_ANGLE = 30
export(float) var SPEED_STEER_ANGLE = 10
export(float) var MAX_STEER_SPEED = 120.0
export(float) var MAX_STEER_INPUT = 90.0
export(float) var STEER_SPEED = 1.0

var throttle: float = 0.0 setget _set_throttle
var current_gear: int = 0 setget _set_current_gear
var clutch: float = 1.0 setget _set_clutch

var _engine_rpm := 0.0
var _wheel_rpm := 0.0


func _set_throttle(value):
	throttle = clamp(value, 0.0, 1.0)


func _set_current_gear(value):
	current_gear = clamp(value, 0, GEAR_RATIOS.size() - 1)


func _set_clutch(value):
	clutch = clamp(value, 0.0, 1.0)


func _init():
	._init()
	
	# Look for neutral gear (ratio 0.0), otherwise use first gear in the list.
	current_gear = GEAR_RATIOS.find(0.0)
	if current_gear < 0:
		current_gear = 0


# calculate the RPM of our engine based on the current velocity of our car
#func calculate_rpm() -> float:
#	assert(current_gear >= 0 and current_gear < GEAR_RATIOS.size())
#	assert(rpm_wheel != null)
#	# if we are in neutral, no rpm
#	if current_gear == 0:
#		return 0.0
#
#	var wheel_circumference : float = 2.0 * PI * rpm_wheel.radius
#	var wheel_rotation_speed : float = 60.0 * current_speed_mps / wheel_circumference
#	var drive_shaft_rotation_speed : float = wheel_rotation_speed * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO
#	return drive_shaft_rotation_speed

func _compute_wheel_torque(state):
	assert(current_gear >= 0 and current_gear < GEAR_RATIOS.size())
	
	var rpm_factor = _engine_rpm / MAX_ENGINE_RPM
	var power_factor = ENGINE_TORQUE_CURVE.interpolate_baked(rpm_factor)
	var engine_torque = throttle * power_factor * MAX_ENGINE_TORQUE
	var transmission_torque = engine_torque * GEAR_RATIOS[current_gear - 1] * FINAL_DRIVE_RATIO
	var clutch_torque = clutch * transmission_torque
	
	for wheel in _wheels:
		if !wheel.use_as_traction:
			continue

		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
#		if raycast.is_in_contact:
#			var contact_rpm = 

	wheel_torque = 0.0
