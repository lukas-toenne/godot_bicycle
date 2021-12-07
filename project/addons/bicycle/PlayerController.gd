extends VehicleController
class_name PlayerController

export(bool) var start_paused = false

export(float) var rotation_speed = 1.0
export(float) var yaw_factor = 0.005
export(float) var pitch_factor = 0.005
export(bool) var pitch_inverted = false

export(float) var camdist_default = 10.0
export(float) var camdist_min = 2.0
export(float) var camdist_max = 50.0
export(float, 0.0, 1.0) var camdist_zoom_speed = 0.333

var _do_single_step = false
var camera: Camera = Camera.new()
var camera_arm: SpringArm = SpringArm.new()

func _ready():
	# Don't pause camera flying
	pause_mode = Node.PAUSE_MODE_PROCESS
	camera.pause_mode = Node.PAUSE_MODE_PROCESS
	camera_arm.pause_mode = Node.PAUSE_MODE_PROCESS
	get_tree().paused = start_paused

	camera.transform.origin = Vector3(0, 0, camdist_default)
	camera_arm.spring_length = camdist_default
	camera_arm.transform.basis = Basis(Vector3(deg2rad(-30), deg2rad(180), 0))
	camera_arm.add_child(camera)
	# TODO: Disable collision for now, should only really exclude the vehicle
	camera_arm.collision_mask = 0

	camera.make_current()


func _enter_tree():
	get_tree().root.call_deferred("add_child", camera_arm)


func _exit_tree():
	camera_arm.get_parent().call_deferred("remove_child", camera_arm)


func enter_vehicle(value):
	.enter_vehicle(value)


func exit_vehicle():
	.exit_vehicle()


func _input(event):
	# Enable view control with RMB
	if event is InputEventMouseButton and event.button_index == BUTTON_RIGHT:
		if event.pressed:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)

	# Camera rotation
	if Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED:
		if event is InputEventMouseMotion:
			var delta_yaw = -event.relative[0] * yaw_factor * rotation_speed
			var delta_pitch = -event.relative[1] * pitch_factor * rotation_speed
			if pitch_inverted:
				delta_pitch = -delta_pitch
			
			var current = camera_arm.transform.basis.get_euler()
			current.x = clamp(current.x + delta_pitch, -.5 * PI, .5 * PI)
			current.y = fmod(current.y + delta_yaw, 2 * PI)
			camera_arm.transform.basis = Basis(current)

	# Camera zoom
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_WHEEL_UP:
			camera_arm.spring_length = clamp(camera_arm.spring_length / (1.0 + camdist_zoom_speed), camdist_min, camdist_max)
		if event.button_index == BUTTON_WHEEL_DOWN:
			camera_arm.spring_length = clamp(camera_arm.spring_length * (1.0 + camdist_zoom_speed), camdist_min, camdist_max)

	
#		if event is InputEventMouseButton:
#			if event.button_index == BUTTON_LEFT and event.pressed:
#				var ball = BallProbe.instance()
#				ball.transform = transform
#				ball.linear_velocity = -transform.basis.z * 10.0
#				get_tree().root.add_child(ball)
	
	# Pause and stepping
	if event is InputEventKey:
		if event.scancode == KEY_P and event.pressed:
			get_tree().paused = !get_tree().paused
			_do_single_step = false
		if event.scancode == KEY_T and event.pressed:
			_do_single_step = true
			get_tree().paused = false

	if event.is_action("move_forward") or event.is_action("move_back"):
		var move_forward = event.get_action_strength("move_forward")
		var move_back = event.get_action_strength("move_back")
		forward_movement = move_forward - move_back
	if event.is_action("move_right") or event.is_action("move_left"):
		var move_right = event.get_action_strength("move_right")
		var move_left = event.get_action_strength("move_left")
		side_movement = move_right - move_left
	if event.is_action("turn_right") or event.is_action("turn_left"):
		var turn_right = event.get_action_strength("turn_right")
		var turn_left = event.get_action_strength("turn_left")
		turn = turn_right - turn_left
	if event.is_action("brake"):
		brake = event.get_action_strength("brake")
	if event.is_action("boost"):
		boost = event.get_action_strength("boost")


func _process(delta):
	if _do_single_step:
		_do_single_step = false
		get_tree().paused = true

	if vehicle:
		camera_arm.transform.origin = vehicle.transform.origin
		
		DebugEventRecorder.record_vector(self, "forward", vehicle.transform.origin + Vector3(0, 1.5, 0), vehicle.transform.basis * Vector3(-turn, 0, forward_movement))
