extends Control

onready var _grid := $PanelContainer/ScrollContainer/GridContainer

var _need_reload := true

var _tests = {
	"Stand on plane": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Inclined plane": preload("res://test_vehicles/cases/case_inclined_plane.tscn"),
	"Fall from height": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Upside down": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Dynamic body rubble": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Kinematic pistons": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Bombarding": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
	"Wall crash": preload("res://test_vehicles/cases/case_stand_on_plane.tscn"),
}

var _vehicles = {
	"Car": preload("res://test_vehicles/vehicles/car.tscn"),
	"Truck": preload("res://test_vehicles/vehicles/car.tscn"),
	"Three-wheeler": preload("res://test_vehicles/vehicles/car.tscn"),
	"Bulldozer": preload("res://test_vehicles/vehicles/car.tscn"),
	"Steamroller": preload("res://test_vehicles/vehicles/car.tscn"),
	"Motorcycle": preload("res://test_vehicles/vehicles/car.tscn"),
	"Bicycle": preload("res://test_vehicles/vehicles/car.tscn"),
	"Unicycle": preload("res://test_vehicles/vehicles/car.tscn"),
	"Skateboard": preload("res://test_vehicles/vehicles/car.tscn"),
}

var _environment = preload("res://test_vehicles/cases/test_environment.tres")


func _ready():
	if _need_reload:
		reload_tests()


func reload_tests():
	_grid.columns = 1 + _tests.size()

	# Empty placeholder
	var empty = Control.new()
	_grid.add_child(empty)

	# Test labels
	for test_name in _tests:
		var label = Label.new()
		label.text = test_name
		label.autowrap = true
		_grid.add_child(label)

	# Test worlds and viewports
	for vehicle_name in _vehicles:
		# Vehicle label
		var label = Label.new()
		label.text = vehicle_name
#		label.autowrap = true
		_grid.add_child(label)

		for test_name in _tests:
			var vp_cont = ViewportContainer.new()
			_grid.add_child(vp_cont)

			var vp = Viewport.new()
			vp.size = Vector2(128, 128)
			vp.own_world = true
			vp_cont.add_child(vp)

			var scene = _tests[test_name].instance()
			assert(scene != null)
			vp.add_child(scene)
			
			var world_env = WorldEnvironment.new()
			world_env.environment = _environment
			scene.add_child(world_env)
			
			var camera = Camera.new()
			camera.look_at_from_position(Vector3(4, 3, 2), Vector3(0, 0, 0), Vector3(0, 1, 0))
			scene.add_child(camera)

			var vehicle = _vehicles[vehicle_name].instance()
			assert(vehicle != null)
			vehicle.transform.origin = Vector3(0, 2, 0)
			scene.add_child(vehicle)

#			var button = Button.new()
#			button.text = vehicle
#			vp_cont.add_child(button)

	_need_reload = false
