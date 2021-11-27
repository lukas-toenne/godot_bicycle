#pragma once

#include <Godot.hpp>
#include <Sprite.hpp>

namespace godot {

class BicycleVehicle : public Sprite {
    GODOT_CLASS(BicycleVehicle, Sprite)

private:
    float time_passed;

public:
    static void _register_methods();

    BicycleVehicle();
    ~BicycleVehicle();

    void _init(); // our initializer called by Godot

    void _process(float delta);
};

}
