extends RigidBody
class_name Bicycle

signal wheels_changed

export(float) var SIDE_FRICTION = 0.2

# Torque applied to traction wheels from engine [Nm]
export(float) var wheel_torque = 0.0
# Friction brake torque per unit of angular velocity [Nms]
export(float) var wheel_brake = 0.0
export(float) var steering_angle = 0.0

var controller setget _set_controller
var _wheels := Dictionary()

var ragdoll: bool setget _set_ragdoll
func _set_ragdoll(value):
	ragdoll = value
	if ragdoll:
		set_physics_process(false)
#		_rig.physical_bones_start_simulation(["Frame", "FrontSprocket", "SteeringAxis"])
	else:
#		_rig.physical_bones_stop_simulation()
		set_physics_process(true)


func _set_controller(value):
	controller = value
	_set_ragdoll(controller == null)


func add_wheel(wheel: BicycleWheel):
	assert(!_wheels.has(wheel))
	_wheels[wheel] = wheel
	emit_signal("wheels_changed")


func remove_wheel(wheel: BicycleWheel):
	assert(_wheels.has(wheel))
	_wheels.erase(wheel)
	emit_signal("wheels_changed")


func _ready():
	_set_ragdoll(true)


func _physics_process(delta):
	# This function should only run when the bicycle is controlled by the player.
	assert(!ragdoll)


func _integrate_forces(state):
	_ray_cast_wheels(state)
	_apply_suspension(state)

	_compute_wheel_torque(state)

	_apply_friction(state)
	_update_wheel_rotation(state)


func _ray_cast_wheels(state: PhysicsDirectBodyState):
	var space := get_world().space
	var space_state := PhysicsServer.space_get_direct_state(space)
	var chassis_transform = state.transform
	
	for wheel in _wheels:
		var wheel_transform = chassis_transform * wheel._rest_transform
		var info: BicycleWheel.RaycastInfo = wheel._raycast_info
		
		info.hard_point = wheel_transform.origin
		info.wheel_direction = -wheel_transform.basis.y.normalized()
		info.wheel_axle = wheel_transform.basis.x.normalized()
		
		# TODO For accurate future collision detection use cast_motion:
		#space_state.cast_motion()
		var source = info.hard_point - info.wheel_direction * wheel.radius
		var target = info.hard_point + info.wheel_direction * (wheel.radius + wheel.rest_length)
		var exclude = [self]
		var result = space_state.intersect_ray(source, target, exclude, collision_mask)

		if result.empty():
			info.is_in_contact = false
			info.ground_object = null
			info.contact_point = target
			info.contact_normal = -info.wheel_direction
			info.suspension_length = wheel.rest_length

			info.relative_contact_velocity = Vector3(0, 0, 0)
			info.suspension_relative_velocity = 0.0
			info.clipped_inv_contact_dot_suspension = 1.0
		else:
			info.is_in_contact = true
			info.ground_object = result["collider"] as PhysicsBody
			info.contact_point = result["position"]
			info.contact_normal = result["normal"]
			
			var distance = source.distance_to(info.contact_point)
#			var param = distance / (2.0 * wheel.radius + wheel.rest_length)
			info.suspension_length = clamp(distance - 2.0 * wheel.radius, wheel.rest_length - wheel.travel, wheel.rest_length + wheel.travel)
			
			# Velocity of the chassis relative to the contact point.
			var chassis_vel_at_contact = state.linear_velocity + state.angular_velocity.cross(info.contact_point - state.transform.origin)
			if info.ground_object is RigidBody:
				# Note: DO NOT access the ground object's state from the physics server here,
				# as that will invalidate all future impulse addition!
				info.inv_ground_mass = 1.0 / info.ground_object.mass if info.ground_object.mass > 0.0 else 0.0
				var ground_vel_at_contact = info.ground_object.linear_velocity + info.ground_object.angular_velocity.cross(info.contact_point - info.ground_object.transform.origin)
				info.relative_contact_velocity = chassis_vel_at_contact - ground_vel_at_contact
			else:
				info.inv_ground_mass = 0.0
				info.relative_contact_velocity = chassis_vel_at_contact

			# Wheel direction projected on contact normal (cosine of contact angle with the ground), must be -1..0 for valid contact.
			var proj_wheel_dir = info.contact_normal.dot(info.wheel_direction)
			var contact_threshold = -1.0e-3
			if proj_wheel_dir < contact_threshold:
				var proj_vel = info.contact_normal.dot(chassis_vel_at_contact)
				var inv = -1.0 / proj_wheel_dir
				info.suspension_relative_velocity = proj_vel * inv
				info.clipped_inv_contact_dot_suspension = inv
			else:
				info.suspension_relative_velocity = 0.0
				info.clipped_inv_contact_dot_suspension = -1.0 / contact_threshold

		DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.relative_contact_velocity)
#		DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.contact_normal * info.suspension_relative_velocity)

		DebugEventRecorder.record_spring(wheel, "suspension", info.hard_point, info.wheel_direction, wheel.rest_length, info.suspension_length)
#		if info.is_in_contact:
#			print("Wheel {wheel} in contact: ground={ground}, point={point}, normal={normal}, suspension={suslen}, rel.vel.={relvel}, proj.suspension={invproj}".format({
#				"wheel":wheel,
#				"ground":info.ground_object,
#				"point":info.contact_point,
#				"normal":info.contact_normal,
#				"suslen":info.suspension_length,
#				"relvel":info.suspension_relative_velocity,
#				"invproj":info.clipped_inv_contact_dot_suspension,
#				}))
#		else:
#			print("Wheel {wheel} free [ground={ground}, point={point}, normal={normal}, suspension={suslen}, rel.vel.={relvel}, proj.suspension={invproj}]".format({
#				"wheel":wheel,
#				"ground":info.ground_object,
#				"point":info.contact_point,
#				"normal":info.contact_normal,
#				"suslen":info.suspension_length,
#				"relvel":info.suspension_relative_velocity,
#				"invproj":info.clipped_inv_contact_dot_suspension,
#				}))


func _apply_suspension(state: PhysicsDirectBodyState):
	var chassis_mass := mass
	
	for wheel in _wheels:
		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
		
		var suspension_force := 0.0
		if raycast.is_in_contact:
			# Dampened spring forces from suspension
			var spring_extension = wheel.rest_length - raycast.suspension_length
			var stiffness = wheel.stiffness * spring_extension * raycast.clipped_inv_contact_dot_suspension
			var damping = -raycast.suspension_relative_velocity
			if damping > 0.0:
				damping *= wheel.compression_damping
			else:
				damping *= wheel.relaxation_damping

			# Negative force would mean the wheel sticks to the ground when lifted up.
			suspension_force = clamp((stiffness + damping) * chassis_mass, 0.0, wheel.max_force)

			var impulse = raycast.contact_normal * suspension_force * state.step;
			var relative_position = raycast.contact_point - state.transform.origin;
			state.apply_impulse(relative_position, impulse)
			DebugEventRecorder.record_vector(wheel, "impulse", raycast.contact_point, impulse)


# This function is implemented by subclasses
func _compute_wheel_torque(state):
	wheel_torque = 0.0


func _apply_friction(state: PhysicsDirectBodyState):
	var inv_chassis_mass := 1.0 / mass if mass > 0.0 else 0.0

	for wheel in _wheels:
		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
		
		if raycast.is_in_contact:
			var normal := raycast.contact_normal
			var forward := normal.cross(raycast.wheel_axle).normalized()
			var axle := forward.cross(normal)
			
			var forward_impulse := 0.0
			var side_impulse := 0.0
			
			var max_rolling_friction = max(wheel_brake, 0.0)
			forward_impulse += _calc_rolling_friction(wheel, max_rolling_friction)
			if wheel.torque != 0.0:
				forward_impulse -= wheel.torque * wheel.radius * state.step
			
			var inv_effective_mass = inv_chassis_mass + raycast.inv_ground_mass
			var effective_mass = 1.0/inv_effective_mass if inv_effective_mass > 0.0 else 0.0
			side_impulse = -SIDE_FRICTION * effective_mass * raycast.relative_contact_velocity.dot(axle)
			
			var relative_position = raycast.contact_point - state.transform.origin;
			if forward_impulse != 0.0:
				state.apply_impulse(relative_position, forward_impulse * forward)
			if side_impulse != 0.0:
				state.apply_impulse(relative_position, side_impulse * axle)


# NOTE: Both Bullet and Godot code for this is very old and without comment.
# What is happening here is that the penetration and friction constraints are
# solved using an implicit method, with an approximate Jacobian inversion.
# The approach is described quite well in Daniel Chappuis
#   "Constraints Derivation for Rigid Body Simulation in 3D"
# https://danielchappuis.ch/download/ConstraintsDerivationRigidBody3D.pdf
func _calc_rolling_friction(wheel: BicycleWheel, max_impulse: float):
	var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
	var normal := raycast.contact_normal
	var forward := normal.cross(raycast.wheel_axle).normalized()
	
	var forward_velocity = raycast.relative_contact_velocity.dot(forward)

	var impulse = forward_velocity * 
	return clamp(impulse, -max_impulse, max_impulse)


# Solves bilateral constraint between the wheel and the ground object.
# This takes into account the suspension spring between vehicle body and wheel,
# as well as the friction between wheel and ground object.
# The dynamic equations between these objects are solved implicitly,
# resulting in the impulses on all 3 objects required to satisfy the constraint.
func _solve_wheel_contact(wheel: BicycleWheel):
	# TODO
	pass


func _update_wheel_rotation(state: PhysicsDirectBodyState):
	pass
