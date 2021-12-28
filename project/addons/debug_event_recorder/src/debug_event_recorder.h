#pragma once

#include <Godot.hpp>
#include <Node.hpp>

namespace godot {

class DebugEventRecorder : public Node {
    GODOT_CLASS(DebugEventRecorder, Node)

private:
    float time_passed;

public:
    static void _register_methods();

    DebugEventRecorder();
    ~DebugEventRecorder();

    void _init(); // our initializer called by Godot

    void _process(float delta);
};

}
