#include "bicycle_vehicle.h"

using namespace godot;

void BicycleVehicle::_register_methods() {
    register_method("_process", &BicycleVehicle::_process);
}

BicycleVehicle::BicycleVehicle() {
}

BicycleVehicle::~BicycleVehicle() {
    // add your cleanup here
}

void BicycleVehicle::_init() {
    // initialize any variables here
    time_passed = 0.0;
}

void BicycleVehicle::_process(float delta) {
    time_passed += delta;

    Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 10.0 + (10.0 * cos(time_passed * 1.5)));

    set_position(new_position);
}
