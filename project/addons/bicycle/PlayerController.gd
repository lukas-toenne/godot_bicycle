extends VehicleController
class_name PlayerController


func _input(event):
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
	if vehicle:
		DebugEventRecorder.record_vector(self, "forward", vehicle.transform.origin + Vector3(0, 1.5, 0), vehicle.transform.basis * Vector3(-turn, 0, forward_movement))
