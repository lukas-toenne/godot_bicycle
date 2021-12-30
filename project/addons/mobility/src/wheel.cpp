#include "wheel.h"

#include "chassis.h"

using namespace godot;

void Wheel::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_radius"), &Wheel::get_radius);
	ClassDB::bind_method(D_METHOD("set_radius", "value"), &Wheel::set_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius"), "set_radius", "get_radius");

	ClassDB::bind_method(D_METHOD("get_suspension_rest_length"), &Wheel::get_suspension_rest_length);
	ClassDB::bind_method(D_METHOD("set_suspension_rest_length", "value"), &Wheel::set_suspension_rest_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_rest_length"), "set_suspension_rest_length", "get_suspension_rest_length");

	ClassDB::bind_method(D_METHOD("get_suspension_stiffness"), &Wheel::get_suspension_stiffness);
	ClassDB::bind_method(D_METHOD("set_suspension_stiffness", "value"), &Wheel::set_suspension_stiffness);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_stiffness"), "set_suspension_stiffness", "get_suspension_stiffness");

	ClassDB::bind_method(D_METHOD("get_suspension_damping"), &Wheel::get_suspension_damping);
	ClassDB::bind_method(D_METHOD("set_suspension_damping", "value"), &Wheel::set_suspension_damping);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_damping"), "set_suspension_damping", "get_suspension_damping");

	ClassDB::bind_method(D_METHOD("get_torque"), &Wheel::get_torque);
	ClassDB::bind_method(D_METHOD("set_torque", "value"), &Wheel::set_torque);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque"), "set_torque", "get_torque");
}

Wheel::Wheel() {
	m_radius = 0.5f;
	m_suspension_rest_length = 0.15f;
	m_suspension_stiffness = 6.0f;
	m_suspension_damping = 0.85f;
	m_torque = 0.0f;
}

Wheel::~Wheel() {
}

void Wheel::_enter_tree()
{
	if (Chassis* chassis = Object::cast_to<Chassis>(get_parent()))
	{
		chassis->add_wheel(this);
	}
}

void Wheel::_exit_tree()
{
	if (Chassis* chassis = Object::cast_to<Chassis>(get_parent()))
	{
		chassis->remove_wheel(this);
	}
}

void Wheel::set_radius(float radius)
{
	m_radius = radius;
}

void Wheel::set_suspension_rest_length(float suspension_rest_length)
{
	m_suspension_rest_length = suspension_rest_length;
}

void Wheel::set_suspension_stiffness(float suspension_stiffness)
{
	m_suspension_stiffness = suspension_stiffness;
}

void Wheel::set_suspension_damping(float suspension_damping)
{
	m_suspension_damping = suspension_damping;
}

void Wheel::set_torque(float torque)
{
	m_torque = torque;
}
