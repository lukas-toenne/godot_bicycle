extends RigidBody
class_name Bicycle

signal wheels_changed

export(bool) var pause_on_first_contact = true

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
#	_apply_suspension(state)

	_compute_wheel_torque(state)
	
	_solve_wheel_constraints(state)

#	_apply_friction(state)
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
		DebugEventRecorder.record_vector(wheel, "raycast", source, target - source)
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
			if pause_on_first_contact:
				pause_on_first_contact = false
				get_tree().paused = true
			
			info.is_in_contact = true
			info.ground_object = result["collider"] as PhysicsBody
			info.contact_point = result["position"]
			info.contact_normal = result["normal"]
			
			var distance = source.distance_to(info.contact_point)
#			var param = distance / (2.0 * wheel.radius + wheel.rest_length)
			info.suspension_length = clamp(distance - 2.0 * wheel.radius, wheel.rest_length - wheel.travel, wheel.rest_length + wheel.travel)
			
			# Velocity of the chassis relative to the contact point.
			var chassis_vel_at_contact = state.linear_velocity + state.angular_velocity.cross(info.contact_point - state.transform.origin)
			# XXX RigidBody still has a mode that can make it static or kinematic, have to check and handle accordingly!
			if info.ground_object is RigidBody:
				# Note: DO NOT access the ground object's state from the physics server here,
				# as that will invalidate all future impulse addition!
				info.inv_ground_mass = 1.0 / info.ground_object.mass if info.ground_object.mass > 0.0 else 0.0
				info.inv_ground_inertia = info.ground_object.get_inverse_inertia_tensor()
				info.ground_linear_velocity = info.ground_object.linear_velocity
				info.ground_angular_velocity = info.ground_object.angular_velocity
				var ground_vel_at_contact = info.ground_object.linear_velocity + info.ground_object.angular_velocity.cross(info.contact_point - info.ground_object.transform.origin)
				info.relative_contact_velocity = chassis_vel_at_contact - ground_vel_at_contact
			else:
				info.inv_ground_mass = 0.0
				info.inv_ground_inertia = Basis(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0))
				info.ground_linear_velocity = Vector3(0, 0, 0)
				info.ground_angular_velocity = Vector3(0, 0, 0)
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

#		DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.relative_contact_velocity)
#		DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.contact_normal * info.suspension_relative_velocity)

		if info.is_in_contact:
			DebugEventRecorder.record_spring(wheel, "suspension", info.hard_point, info.wheel_direction, wheel.rest_length, info.suspension_length)
#			print("Wheel {wheel} in contact: ground={ground}, point={point}, normal={normal}, suspension={suslen}, rel.vel.={relvel}, proj.suspension={invproj}".format({
#				"wheel":wheel,
#				"ground":info.ground_object,
#				"point":info.contact_point,
#				"normal":info.contact_normal,
#				"suslen":info.suspension_length,
#				"relvel":info.suspension_relative_velocity,
#				"invproj":info.clipped_inv_contact_dot_suspension,
#				}))
		else:
			DebugEventRecorder.clear_event(wheel, "suspension")
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
# solved using an implicit method with Baumgarte stabilization and approximate
# Jacobian inversion.
#
# Some educational resources:
# - Daniel Chappuis, "Constraints Derivation for Rigid Body Simulation in 3D"
#   https://danielchappuis.ch/download/ConstraintsDerivationRigidBody3D.pdf
# - Ming-Lun Chou, Physics Constraints in Games
#   https://www.youtube.com/watch?v=MTVdBgQY9LY
#   (Part 2 explains contact constraints specifically)
#
# TL;DR steps:
# 1) The constraint function C describes the condition that should be satisfied:
#      C(X, V, R, W) = 0   [X: position, V: velocity, R: orientation, W: angular velocity]
# 2) Compute derivative (Jacobian) and set to zero to enforce the constraint.
#      J * Q = dC/dq * Q = 0   [Q is combined state vector (X, V, R, W)]
# 3) Setting J to zero only enforces that C is constant, but it can still have an offset.
#    To make sure it returns to zero, add a bias factor b:
#      J * Q + b = 0
#    Baumgarte stabilization defines b as
#      b = beta / h   [h: time step, beta: correction factor in 0..1]
#    Correction factor can be defined in terms of spring damping (see below).
# 4) Impulse correction: To correct velocity such that the constraint is valid,
#    apply a correctional impulse M * dV. A Lagrange multiplier is used to
#    solve for dV:
#      M * (V1 - V0) = J^T * lambda   [M: diagonal mass matrix, V0: unconstraint velocity, V1: corrected velocity]
# 5) Lagrange multiplier lambda is expressed through effective mass:
#      M_eff = (J * M^(-1) * J^T)^(-1)
#      lambda = -M_eff * (J * V0 + b)

func _calc_rolling_friction(wheel: BicycleWheel, max_impulse: float):
	var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
	var normal := raycast.contact_normal
	var forward := normal.cross(raycast.wheel_axle).normalized()
	
	var forward_velocity = raycast.relative_contact_velocity.dot(forward)

	# TODO
	var impulse = forward_velocity * 0.0
	return clamp(impulse, -max_impulse, max_impulse)


# Solves bilateral constraint between the wheel and the ground object.
# This takes into account the suspension spring between vehicle body and wheel,
# as well as the friction between wheel and ground object.
func _solve_wheel_constraints(state: PhysicsDirectBodyState):
	# Inverse of the effective mass.
	# Symmetric positive-definite matrix with a row/column for each constraint.
	# Each constraint adds an inverse effective mass term,
	# at the end the matrix is used to solve for the Laplace multipliers.
#	var inv_M_eff
	
#	print("SOLVE ")
	var wheel_array = _wheels.values()
	var total_lambda = []
	total_lambda.resize(wheel_array.size())
	for k in wheel_array.size():
		total_lambda[k] = 0.0
	for iteration in 1:
#		print("  iteration ", iteration)
		for k in wheel_array.size():
#			if k != 0 and k != 3:
			if k != 3:
				continue
			var wheel = wheel_array[k]
			total_lambda[k] = _solve_single_wheel_constraint(state, wheel, total_lambda[k])
#			print("  wheel ", wheel, " total lambda", total_lambda[k])

			DebugEventRecorder.record_vector(self, "impulse"+str(wheel), state.transform.origin, state.angular_velocity)


func _solve_single_wheel_constraint(state: PhysicsDirectBodyState, wheel: BicycleWheel, total_lambda: float):
	var contact_bias = 1.0

	# Variables to solve for:
	#   Q = [Q_chassis, Q_ground]
	#     = [Vc, Wc,    Vg, Wg]
	var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
	if !raycast.is_in_contact:
		DebugEventRecorder.clear_event(wheel, "impulse")
		DebugEventRecorder.clear_event(wheel, "torque")
		return total_lambda

	var normal := raycast.contact_normal
	var offset_chassis = raycast.contact_point - state.transform.origin
	var offset_ground = raycast.contact_point - raycast.ground_object.transform.origin

	# Jacobian elements for a contact constraint
	var jac_vel_chassis = -normal
	var jac_rot_chassis = -offset_chassis.cross(normal)
	var jac_vel_ground = normal
	var jac_rot_ground = offset_ground.cross(normal)

	# Effective mass
	var inv_m_chassis = state.inverse_mass
	var inv_I_chassis = state.principal_inertia_axes * state.principal_inertia_axes.transposed().scaled(state.inverse_inertia)
	var inv_m_ground = raycast.inv_ground_mass
	var inv_I_ground = raycast.inv_ground_inertia
	var M_eff_vel_ch = jac_vel_chassis.dot(inv_m_chassis * jac_vel_chassis)
	var M_eff_rot_ch = jac_rot_chassis.dot(inv_I_chassis * jac_rot_chassis)
	var M_eff_vel_gr = jac_vel_ground.dot(inv_m_ground * jac_vel_ground)
	var M_eff_rot_gr = jac_rot_ground.dot(inv_I_ground * jac_rot_ground)
	var M_eff = 1.0 / (jac_vel_chassis.dot(inv_m_chassis * jac_vel_chassis) 
					 + jac_rot_chassis.dot(inv_I_chassis * jac_rot_chassis)
					 + jac_vel_ground.dot(inv_m_ground * jac_vel_ground)
					 + jac_rot_ground.dot(inv_I_ground * jac_rot_ground))

	# Lagrange multiplier
	var vel_chassis = state.linear_velocity
	var rot_chassis = state.angular_velocity
	var vel_ground = raycast.ground_linear_velocity
	var rot_ground = raycast.ground_angular_velocity
	var lambda = -M_eff * (jac_vel_chassis.dot(vel_chassis)
						 + jac_rot_chassis.dot(rot_chassis)
						 + jac_vel_ground.dot(vel_ground)
						 + jac_rot_ground.dot(rot_ground)
						 + contact_bias)
	var old_total_lambda = total_lambda
	total_lambda = min(total_lambda + lambda, 0.0)
	lambda = total_lambda - old_total_lambda
#	print("    lambda=", lambda)
	
	# Apply corrective impulse derived from Lagrance multiplier
	var impulse_chassis = jac_vel_chassis * lambda
	var torque_chassis = jac_rot_chassis * lambda
	var impulse_ground = jac_vel_ground * lambda
	var torque_ground = jac_rot_ground * lambda

#	var curvel = state.linear_velocity
	state.apply_central_impulse(impulse_chassis)
#	print(wheel, curvel, " + ", inv_m_chassis * impulse_chassis, " = ", state.linear_velocity)
#	var currot = state.angular_velocity
	state.apply_torque_impulse(torque_chassis)
#	print(wheel, currot, " + ", inv_I_chassis * torque_chassis, " = ", state.angular_velocity)
	var debug_origin = lerp(state.transform.origin, raycast.contact_point, 0.2)
	DebugEventRecorder.record_vector(wheel, "impulse", debug_origin, jac_vel_chassis * total_lambda)
	DebugEventRecorder.record_vector(wheel, "torque", debug_origin, jac_rot_chassis * total_lambda)
	# TODO can we apply impulse to the ground object?
	
	return total_lambda


func _update_wheel_rotation(state: PhysicsDirectBodyState):
	pass
