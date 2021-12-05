extends Spatial

const _numvec = 3
var _vectors := []


func _ready():
	for i in _numvec:
		var vec = DebugVector.new()
		_vectors.append(vec)
		add_child(vec)


func _process(delta):
	_vectors[0].start = Vector3(1, 1, 1)
	_vectors[0].direction = Vector3(1, 0, 0)
	_vectors[1].start = Vector3(1, 1, 1)
	_vectors[1].direction = Vector3(0, 1, 0)
	_vectors[2].start = Vector3(1, 1, 1)
	_vectors[2].direction = Vector3(0, 0, 1)

	DebugEventRecorder.record_vector(self, "A", Vector3(2, 1, 1), Vector3(1, 1, 0))
