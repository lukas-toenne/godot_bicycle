@tool
extends EditorPlugin


func _enter_tree():
	add_autoload_singleton("DebugEventRecorder", "res://addons/debug_event_recorder/DebugEventRecorder.gd")


func _exit_tree():
	remove_autoload_singleton("DebugEventRecorder")
