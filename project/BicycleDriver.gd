extends Node3D


var _bicycle: Bicycle = null


func _enter_bicycle():
	if !_bicycle:
		return
	_bicycle.ragdoll = false


func _exit_bicycle():
	if !_bicycle:
		return
	_bicycle.ragdoll = true


func set_bicycle(bicycle: Bicycle):
	_exit_bicycle()
	_bicycle = bicycle
	_enter_bicycle()
