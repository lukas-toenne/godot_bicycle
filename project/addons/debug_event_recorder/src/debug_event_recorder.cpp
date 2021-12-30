#include "debug_event_recorder.h"

using namespace godot;

void DebugEventRecorder::_bind_methods() {
}

DebugEventRecorder::DebugEventRecorder() {
    time_passed = 0.0f;
}

DebugEventRecorder::~DebugEventRecorder() {
}

void DebugEventRecorder::_process(double delta) {
    time_passed += (float)delta;

    // Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 10.0 + (10.0 * cos(time_passed * 1.5)));

    // set_position(new_position);
}
