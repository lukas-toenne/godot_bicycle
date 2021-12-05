extends VehicleBody
class_name BicycleVehicle

export(float) var balance_stiffness = 10.0
export(float) var balance_damping = 0.0

onready var _rig := $BicycleRig/Skeleton
onready var _steering_center: int = _rig.find_bone("SteeringCenter")

var _balance_joint: Joint = null

var ragdoll: bool setget _set_ragdoll
func _set_ragdoll(value):
	ragdoll = value
	if ragdoll:
		set_physics_process(false)
#		_rig.physical_bones_stop_simulation()
#		$FrontWheelBody.collision_layer = 0
#		$FrontWheelBody.collision_mask = 0
#		$RearWheelBody.collision_layer = 0
#		$RearWheelBody.collision_mask = 0
#		$BicycleRig/Skeleton/pbFrame.collision_layer = 1
#		$BicycleRig/Skeleton/pbFrame.collision_mask = 1
#		$BicycleRig/Skeleton/pbFrontSprocket.collision_layer = 1
#		$BicycleRig/Skeleton/pbFrontSprocket.collision_mask = 1
#		$BicycleRig/Skeleton/pbSteeringAxis.collision_layer = 1
#		$BicycleRig/Skeleton/pbSteeringAxis.collision_mask = 1
#		$BicycleRig/Skeleton/pbFrontWheel.collision_layer = 1
#		$BicycleRig/Skeleton/pbFrontWheel.collision_mask = 1
#		$BicycleRig/Skeleton/pbRearWheel.collision_layer = 1
#		$BicycleRig/Skeleton/pbRearWheel.collision_mask = 1
		_rig.physical_bones_start_simulation(["Frame", "FrontSprocket", "SteeringAxis"])
	else:
		_rig.physical_bones_stop_simulation()
#		_rig.physical_bones_start_simulation(["Frame"])
#		$FrontWheelBody.collision_layer = 1
#		$FrontWheelBody.collision_mask = 1
#		$RearWheelBody.collision_layer = 1
#		$RearWheelBody.collision_mask = 1
#		$BicycleRig/Skeleton/pbFrame.collision_layer = 0
#		$BicycleRig/Skeleton/pbFrame.collision_mask = 0
#		$BicycleRig/Skeleton/pbFrontSprocket.collision_layer = 0
#		$BicycleRig/Skeleton/pbFrontSprocket.collision_mask = 0
#		$BicycleRig/Skeleton/pbSteeringAxis.collision_layer = 0
#		$BicycleRig/Skeleton/pbSteeringAxis.collision_mask = 0
#		$BicycleRig/Skeleton/pbFrontWheel.collision_layer = 0
#		$BicycleRig/Skeleton/pbFrontWheel.collision_mask = 0
#		$BicycleRig/Skeleton/pbRearWheel.collision_layer = 0
#		$BicycleRig/Skeleton/pbRearWheel.collision_mask = 0
		set_physics_process(true)


func _ready():
	_set_ragdoll(true)

#	_balance_joint = Generic6DOFJoint.new()
#	_balance_joint.set_node_b(get_path())
#	_balance_joint.set("linear_limit_x/enabled", false)
#	_balance_joint.set("linear_limit_y/enabled", false)
#	_balance_joint.set("linear_limit_z/enabled", false)
#	_balance_joint.set("angular_limit_x/enabled", false)
#	_balance_joint.set("angular_limit_y/enabled", false)
#	_balance_joint.set("angular_limit_z/enabled", false)
##	_balance_joint.set("angular_limit_z/softness", 100.0)
##	_balance_joint.set("angular_limit_z/restitution", 10.0)
##	_balance_joint.set("angular_limit_z/damping", 100.0)
#	_balance_joint.set("angular_spring_z/enabled", true)
#	_balance_joint.set("angular_spring_z/stiffness", 100.0)
#	_balance_joint.set("angular_spring_z/damping", 1.0)
##	_balance_joint.transform.origin = $RearWheel.transform.origin + Vector3(0, $RearWheel.wheel_radius, 0)
#	_balance_joint.transform.origin = Vector3(0, 0, 0)
#	add_child(_balance_joint)


func _physics_process(delta):
	# This function should only run when the bicycle is controlled by the player.
	assert(!ragdoll)
	

func _integrate_forces(state):
	# Apply force to move the frame into upright position.
#	var current = _rig.get_bone_global_pose(_steering_center)
#	current = _rig.global_transform * current
#	var lateral = current.basis.z
#	var current = state.transform
#	var lateral = current.basis.x
##	var force = lateral.normalized() * asin(lateral.y) * balance_stiffness
#	var force = Vector3(lateral.x, 0, lateral.z).normalized() * lateral.y * balance_stiffness
##	$BicycleRig/Skeleton/pbFrame.apply_impulse(current.origin, force * delta)
#	add_force(force, current.origin)
#	DebugEventRecorder.record_vector(self, "lateral", current.origin, force)

#	var forward = state.transform.basis.z
#	var lateral = state.transform.basis.x
#	var angvel_local = state.transform.basis.transposed() * state.angular_velocity
#	var torque_z = -lateral.y * balance_stiffness 
#	var damp_z = -angvel_local.z * balance_damping
##	if _last_delta > 0.0:
##		var damp_limit = -angvel_local / _last_delta
###		torque_x = max(torque_x, torque_limit.x) if torque_limit.x < 0.0 else min(torque_x, torque_limit.x)
###		torque_y = max(torque_y, torque_limit.y) if torque_limit.y < 0.0 else min(torque_y, torque_limit.y)
##		damp_z = max(damp_z, damp_limit.z) if damp_limit.z < 0.0 else min(damp_z, damp_limit.z)
#	var torque = forward * (torque_z + damp_z)
#	add_torque(torque)
#	DebugEventRecorder.record_vector(self, "lateral", state.transform.origin, lateral)
#	DebugEventRecorder.record_vector(self, "torque", state.transform.origin, torque)

#	var current = state.transform
#	var forward = current.basis.z
#	var roll_old = angular_velocity.dot(forward)
#	angular_velocity -= forward * angular_velocity.dot(forward)
#	var roll_new = angular_velocity.dot(forward)
#	print(roll_old, " -> ", roll_new)
	pass

