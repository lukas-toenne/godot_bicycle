#pragma once

#include <Godot.hpp>
#include <Spatial.hpp>

namespace godot {

class Wheel : public Spatial {
	GODOT_CLASS(Wheel, Spatial)

public:
	float m_radius;
	float m_suspension_rest_length;
	float m_suspension_stiffness;
	float m_suspension_damping;
	float m_torque;

public:
	static void _register_methods();

	Wheel();
	~Wheel();

	void _init();

	void _notification(const int64_t what);

private:
	static const Wheel m_cdo;
};

}
