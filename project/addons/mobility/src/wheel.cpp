#include "wheel.h"

#include "chassis.h"

using namespace godot;

const Wheel Wheel::m_cdo = Wheel();

void Wheel::_register_methods() {
	register_method("_notification", &Wheel::_notification);

	register_property<Wheel, float>("radius", &Wheel::m_radius, m_cdo.m_radius);
	register_property<Wheel, float>("suspension_rest_length", &Wheel::m_suspension_rest_length, m_cdo.m_suspension_rest_length);
	register_property<Wheel, float>("suspension_stiffness", &Wheel::m_suspension_stiffness, m_cdo.m_suspension_stiffness);
	register_property<Wheel, float>("suspension_damping", &Wheel::m_suspension_damping, m_cdo.m_suspension_damping);
	register_property<Wheel, float>("torque", &Wheel::m_torque, m_cdo.m_torque);
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

void Wheel::_init() {
}

void Wheel::_notification(const int64_t what)
{
	if (what == Node::NOTIFICATION_ENTER_TREE)
	{
		if (Chassis* chassis = Object::cast_to<Chassis>(get_parent()))
		{
			chassis->add_wheel(this);
		}
	}
	if (what == Node::NOTIFICATION_EXIT_TREE)
	{
		if (Chassis* chassis = Object::cast_to<Chassis>(get_parent()))
		{
			chassis->remove_wheel(this);
		}
	}
}
