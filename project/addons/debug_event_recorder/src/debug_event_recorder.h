#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

class DebugEventRecorder : public Node {
    GDCLASS(DebugEventRecorder, Node)

private:
    float time_passed;

public:
    static void _bind_methods();

    DebugEventRecorder();
    ~DebugEventRecorder();

    virtual void _process(double delta) override;
};

}
