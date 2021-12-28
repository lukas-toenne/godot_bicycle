extends RigidBody
class_name Bicycle

signal wheels_changed

export(bool) var pause_on_first_contact = false

export(float) var SIDE_FRICTION = 0.2

# Torque applied to traction wheels from engine [Nm]
export(float) var wheel_torque = 0.0
# Friction brake torque per unit of angular velocity [Nms]
export(float) var wheel_brake = 0.0
export(float) var steering_angle = 0.0

var controller setget _set_controller
func _set_controller(value):
	controller = value
	_set_ragdoll(controller == null)

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


class Constraint:
	var bodyA: RID
	var bodyB: RID
	
	var solA: BodySolution
	var solB: BodySolution

	# Jacobian elements for a contact constraint:
	# Velocity elements are the contact surface normal,
	# Rotation elements are the axes perpendicular to the center-of-mass direction.
	# Adding impulse in these directions can satisfy the
	# constraint without adding energy (cf. Lagrange multipliers).
	# For a derivation see e.g.
	# Daniel Chappuis, "Constraints Derivation for Rigid Body Simulation in 3D"
	# https://danielchappuis.ch/download/ConstraintsDerivationRigidBody3D.pdf
	var jac_velA: Vector3
	var jac_rotA: Vector3
	var jac_velB: Vector3
	var jac_rotB: Vector3

	# Combination of velocity and position error that needs to be corrected
	var impulse_error: float

	var linear_componentA: Vector3
	var angular_componentA: Vector3
	var linear_componentB: Vector3
	var angular_componentB: Vector3

	# Effective mass
	# Note: Bullet calls the effective mass "impulse denominator" (computeImpulseDenominator, m_jacDiagABInv)
	#       It also includes a "constraint force mixing" (cfm) term.
	var M_eff: float
	var inv_M_eff: float

	# Total applied impulse, used for clamping.
	var applied_impulse: float

	var delta_velA: Vector3
	var delta_rotA: Vector3
	var delta_velB: Vector3
	var delta_rotB: Vector3


class BodySolution:
	var delta_vel: Vector3 = Vector3(0, 0, 0)
	var delta_rot: Vector3 = Vector3(0, 0, 0)


var body_solutions := Dictionary()


func add_wheel(wheel: BicycleWheel):
	assert(!_wheels.has(wheel))
	_wheels[wheel] = wheel
	emit_signal("wheels_changed")


func remove_wheel(wheel: BicycleWheel):
	assert(_wheels.has(wheel))
	_remove_wheel_constraint(wheel)
	_wheels.erase(wheel)
	emit_signal("wheels_changed")


func _ready():
	_set_ragdoll(true)


func _exit_tree():
	for wheel in _wheels:
		_remove_wheel_constraint(wheel)


func _physics_process(delta):
	# This function should only run when the bicycle is controlled by the player.
	assert(!ragdoll)


func _integrate_forces(state):
	_ray_cast_wheels(state)
	_update_wheel_constraints(state)
	_compute_wheel_torque(state)
#	_solve_wheel_constraints(state)
#	_update_wheel_rotation(state)


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


func _update_wheel_constraints(state: PhysicsDirectBodyState):
	for wheel in _wheels:
		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info

		if raycast.is_in_contact:
			if !wheel.constraint_rid:
				_init_wheel_constraint(state, wheel)
		else:
			if wheel.constraint_rid:
				PhysicsServer.free_rid(wheel.constraint_rid)
				wheel.constraint_rid = RID()


func _init_wheel_constraint(state: PhysicsDirectBodyState, wheel: BicycleWheel):
	var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info

	var up = raycast.contact_normal
	var forward = raycast.wheel_axle.cross(raycast.contact_normal).normalized()
	var left = up.cross(forward)
	var world_transform = Transform(left, up, forward, raycast.contact_point)

	var ridA = get_rid()
	var ridB = raycast.ground_object.get_rid()
	var transformA = (state.transform.affine_inverse() * world_transform).orthonormalized()
	var transformB = (raycast.ground_object.transform.affine_inverse() * world_transform).orthonormalized()
#	var transformA = world_transform
#	var transformB = world_transform
	wheel.constraint_rid = PhysicsServer.joint_create_generic_6dof(ridA, transformA, ridB, transformB)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT, true)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT, true)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT, true)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_MOTOR, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_MOTOR, false)
	PhysicsServer.generic_6dof_joint_set_flag(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_FLAG_ENABLE_MOTOR, false)

	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_LINEAR_LOWER_LIMIT, -1.0e9)
	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_LINEAR_LOWER_LIMIT, -1.0e9)
	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_LINEAR_UPPER_LIMIT, 1.0e9)
	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_LINEAR_UPPER_LIMIT, 1.0e9)

#	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_LINEAR_RESTITUTION, 1.0)

	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_X, PhysicsServer.G6DOF_JOINT_LINEAR_SPRING_DAMPING, 100.0)
	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_Z, PhysicsServer.G6DOF_JOINT_LINEAR_DAMPING, 100.0)
#	PhysicsServer.generic_6dof_joint_set_param(wheel.constraint_rid, Vector3.AXIS_Y, PhysicsServer.G6DOF_JOINT_LINEAR_DAMPING, 100.0)


func _remove_wheel_constraint(wheel: BicycleWheel):
	if wheel.constraint_rid:
		PhysicsServer.free_rid(wheel.constraint_rid)
		wheel.constraint_rid = RID()


#func _apply_suspension(state: PhysicsDirectBodyState):
#	var chassis_mass := mass
#
#	for wheel in _wheels:
#		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
#
#		var suspension_force := 0.0
#		if raycast.is_in_contact:
#			# Dampened spring forces from suspension
#			var spring_extension = wheel.rest_length - raycast.suspension_length
#			var stiffness = wheel.stiffness * spring_extension * raycast.clipped_inv_contact_dot_suspension
#			var damping = -raycast.suspension_relative_velocity
#			if damping > 0.0:
#				damping *= wheel.compression_damping
#			else:
#				damping *= wheel.relaxation_damping
#
#			# Negative force would mean the wheel sticks to the ground when lifted up.
#			suspension_force = clamp((stiffness + damping) * chassis_mass, 0.0, wheel.max_force)
#
#			var impulse = raycast.contact_normal * suspension_force * state.step;
#			var relative_position = raycast.contact_point - state.transform.origin;
#			state.apply_impulse(relative_position, impulse)
#			DebugEventRecorder.record_vector(wheel, "impulse", raycast.contact_point, impulse)


# This function is implemented by subclasses
func _compute_wheel_torque(state):
	wheel_torque = 0.0


#func _apply_friction(state: PhysicsDirectBodyState):
#	var inv_chassis_mass := 1.0 / mass if mass > 0.0 else 0.0
#
#	for wheel in _wheels:
#		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
#
#		if raycast.is_in_contact:
#			var normal := raycast.contact_normal
#			var forward := normal.cross(raycast.wheel_axle).normalized()
#			var axle := forward.cross(normal)
#
#			var forward_impulse := 0.0
#			var side_impulse := 0.0
#
#			var max_rolling_friction = max(wheel_brake, 0.0)
#			forward_impulse += _calc_rolling_friction(wheel, max_rolling_friction)
#			if wheel.torque != 0.0:
#				forward_impulse -= wheel.torque * wheel.radius * state.step
#
#			var inv_effective_mass = inv_chassis_mass + raycast.inv_ground_mass
#			var effective_mass = 1.0/inv_effective_mass if inv_effective_mass > 0.0 else 0.0
#			side_impulse = -SIDE_FRICTION * effective_mass * raycast.relative_contact_velocity.dot(axle)
#
#			var relative_position = raycast.contact_point - state.transform.origin;
#			if forward_impulse != 0.0:
#				state.apply_impulse(relative_position, forward_impulse * forward)
#			if side_impulse != 0.0:
#				state.apply_impulse(relative_position, side_impulse * axle)


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

#func _calc_rolling_friction(wheel: BicycleWheel, max_impulse: float):
#	var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
#	var normal := raycast.contact_normal
#	var forward := normal.cross(raycast.wheel_axle).normalized()
#
#	var forward_velocity = raycast.relative_contact_velocity.dot(forward)
#
#	# TODO
#	var impulse = forward_velocity * 0.0
#	return clamp(impulse, -max_impulse, max_impulse)


# Solves bilateral constraint between the wheel and the ground object.
# This takes into account the suspension spring between vehicle body and wheel,
# as well as the friction between wheel and ground object.
func _solve_wheel_constraints(state: PhysicsDirectBodyState):
	var wheel_array = _wheels.values()
	
	body_solutions.clear()

	var constraints = []
	constraints.resize(wheel_array.size())
	for k in constraints.size():
		var wheel = wheel_array[k]
		var raycast: BicycleWheel.RaycastInfo = wheel._raycast_info
		if !raycast.is_in_contact:
			continue
		
		var constraint := Constraint.new()
		_init_constraint(constraint, state, wheel, raycast)
		constraints[k] = constraint
	
	
#	print("SOLVE %d" % [constraints.size()])
	var max_iterations = 50
	var least_squares_residual_threshold = 0.001
	for iteration in max_iterations:
#		print("  iteration ", iteration)
		var least_squares_residual = 0.0
		for k in constraints.size():
			if constraints[k] == null:
				continue
#			if k != 0 and k != 3:
#			if k != 3:
#				continue
			var residual = _resolve_single_constraint(constraints[k])
			least_squares_residual = max(least_squares_residual, residual * residual)
#			print("wheel ", wheel_array[k], " residual=", residual)
#			print("  wheel ", wheel, " total lambda", total_lambda[k])

#			DebugEventRecorder.record_vector(self, "impulse"+str(wheel), state.transform.origin, state.angular_velocity)
		if least_squares_residual <= least_squares_residual_threshold:
#			print("%d iterations, residual=%f" % [iteration + 1, least_squares_residual])
			break
#		if iteration == max_iterations + 1:
#			print("MAX ITERATIONS REACHED")

	if body_solutions.has(get_rid()):
		state.linear_velocity += body_solutions[get_rid()].delta_vel
		state.angular_velocity += body_solutions[get_rid()].delta_rot
		DebugEventRecorder.record_vector(self, "linear_impulse", state.transform.origin, body_solutions[get_rid()].delta_vel)
		DebugEventRecorder.record_vector(self, "angular_impulse", state.transform.origin, body_solutions[get_rid()].delta_rot)


const cfm = 0.0
func _init_constraint(constraint: Constraint, state: PhysicsDirectBodyState, wheel: BicycleWheel, raycast: BicycleWheel.RaycastInfo):
	constraint.bodyA = get_rid()
	constraint.bodyB = raycast.ground_object.get_rid()
	
	# Cached body solution that can be accessed by multiple constraints
	constraint.solA = body_solutions.get(constraint.bodyA, null)
	if constraint.solA == null:
		constraint.solA = BodySolution.new()
		body_solutions[constraint.bodyA] = constraint.solA
	constraint.solB = body_solutions.get(constraint.bodyB, null)
	if constraint.solB == null:
		constraint.solB = BodySolution.new()
		body_solutions[constraint.bodyB] = constraint.solB
	
	var normal := raycast.contact_normal
	var offsetA = raycast.contact_point - state.transform.origin
	var offsetB = raycast.contact_point - raycast.ground_object.transform.origin
	constraint.jac_velA = -normal
	constraint.jac_rotA = -offsetA.cross(normal)
	constraint.jac_velB = normal
	constraint.jac_rotB = offsetB.cross(normal)
	
	var inv_mA = state.inverse_mass
	var inv_IA = state.principal_inertia_axes * state.principal_inertia_axes.transposed().scaled(state.inverse_inertia)
	# XXX RigidBody still has a mode that can make it static or kinematic, have to check and handle accordingly!
	var inv_mB = 0.0
	var inv_IB = Basis(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0))
	if raycast.ground_object is RigidBody:
		# Note: DO NOT access the ground object's state from the physics server here,
		# as that will invalidate all future impulse addition!
		inv_mB = 1.0 / raycast.ground_object.mass if raycast.ground_object.mass > 0.0 else 0.0
		inv_IB = raycast.ground_object.get_inverse_inertia_tensor()

	constraint.linear_componentA = -(inv_mA * constraint.jac_velA)
	constraint.angular_componentA = -(inv_IA * constraint.jac_rotA)
	constraint.linear_componentB = -(inv_mB * constraint.jac_velB)
	constraint.angular_componentB = -(inv_IB * constraint.jac_rotB)

	constraint.inv_M_eff = constraint.jac_velA.dot(inv_mA * constraint.jac_velA) \
						 + constraint.jac_rotA.dot(inv_IA * constraint.jac_rotA) \
						 + constraint.jac_velB.dot(inv_mB * constraint.jac_velB) \
						 + constraint.jac_rotB.dot(inv_IB * constraint.jac_rotB)
	constraint.M_eff = 1.0 / constraint.inv_M_eff if constraint.inv_M_eff > 0.0 else 0.0
	
	# TODO Implement warm starting: Cache contact constraints and use the
	# previous frame's impulse as a starting point so fewer iterations
	# are needed to reach stability.
	constraint.applied_impulse = 0.0
	
	var velA = PhysicsServer.body_get_state(constraint.bodyA, PhysicsServer.BODY_STATE_LINEAR_VELOCITY)
	var rotA = PhysicsServer.body_get_state(constraint.bodyA, PhysicsServer.BODY_STATE_ANGULAR_VELOCITY)
	var velB = PhysicsServer.body_get_state(constraint.bodyB, PhysicsServer.BODY_STATE_LINEAR_VELOCITY)
	var rotB = PhysicsServer.body_get_state(constraint.bodyB, PhysicsServer.BODY_STATE_ANGULAR_VELOCITY)

	# TODO Godot has very convoluted override options for local gravity,
	# this is just the basic version. We can read precomputed gravity from the body state for A,
	# but no way to get corresponding gravity for B in case it's a rigid body too.
	velA += state.total_gravity * inv_mA * state.step

	var nvelA = constraint.jac_velA.dot(velA) + constraint.jac_rotA.dot(rotA)
	var nvelB = constraint.jac_velB.dot(velB) + constraint.jac_rotB.dot(rotB)
	var rel_vel = nvelB - nvelA
	
	# TODO
	var restitution = 0.0
	
	# TODO
	var position_error = 0.0
	var velocity_error = restitution - rel_vel
	constraint.impulse_error = (position_error + velocity_error) * constraint.M_eff


func _resolve_single_constraint(constraint: Constraint):
	var contact_bias = 0.05
	
	var delta_impulse = constraint.impulse_error - constraint.applied_impulse * cfm
	var nvelA = constraint.jac_velA.dot(constraint.solA.delta_vel) + constraint.jac_rotA.dot(constraint.solA.delta_rot)
	var nvelB = constraint.jac_velB.dot(constraint.solB.delta_vel) + constraint.jac_rotB.dot(constraint.solB.delta_rot)
	var baumgarte = 0.0
	delta_impulse += (nvelA - nvelB + baumgarte) * constraint.M_eff
	
	var total_impulse = delta_impulse + constraint.applied_impulse
	# TODO could be something else for friction constraints
	var lower_limit = 0.0
	if total_impulse < lower_limit:
		delta_impulse = lower_limit - constraint.applied_impulse
		constraint.applied_impulse = lower_limit
	else:
		constraint.applied_impulse = total_impulse
	
	constraint.solA.delta_vel += constraint.linear_componentA * delta_impulse
	constraint.solA.delta_rot += constraint.angular_componentA * delta_impulse
	constraint.solB.delta_vel += constraint.linear_componentB * delta_impulse
	constraint.solB.delta_rot += constraint.angular_componentB * delta_impulse
	
	return delta_impulse * constraint.inv_M_eff

#static btScalar gResolveSingleConstraintRowLowerLimit_scalar_reference(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
#{
#	btScalar deltaImpulse = c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm;
#	const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(bodyA.internalGetDeltaLinearVelocity()) + c.m_relpos1CrossNormal.dot(bodyA.internalGetDeltaAngularVelocity());
#	const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(bodyB.internalGetDeltaLinearVelocity()) + c.m_relpos2CrossNormal.dot(bodyB.internalGetDeltaAngularVelocity());
#
#	deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
#	deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
#	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
#	if (sum < c.m_lowerLimit)
#	{
#		deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
#		c.m_appliedImpulse = c.m_lowerLimit;
#	}
#	else
#	{
#		c.m_appliedImpulse = sum;
#	}
#	bodyA.internalApplyImpulse(c.m_contactNormal1 * bodyA.internalGetInvMass(), c.m_angularComponentA, deltaImpulse);
#	bodyB.internalApplyImpulse(c.m_contactNormal2 * bodyB.internalGetInvMass(), c.m_angularComponentB, deltaImpulse);
#
#	return deltaImpulse * (1. / c.m_jacDiagABInv);
#}


func _update_wheel_rotation(state: PhysicsDirectBodyState):
	pass
