#pragma once

#include <godot_cpp/classes/node3d.hpp>

namespace godot {

class Wheel : public Node3D {
	GDCLASS(Wheel, Node3D)

public:
	float m_radius;
	float m_suspension_rest_length;
	float m_suspension_stiffness;
	float m_suspension_damping;
	float m_torque;

public:
	static void _bind_methods();

	Wheel();
	~Wheel();

	virtual void _enter_tree() override;
	virtual void _exit_tree() override;

	float get_radius() const { return m_radius; }
	void set_radius(float radius);

	float get_suspension_rest_length() const { return m_suspension_rest_length; }
	void set_suspension_rest_length(float suspension_rest_length);

	float get_suspension_stiffness() const { return m_suspension_stiffness; }
	void set_suspension_stiffness(float suspension_stiffness);

	float get_suspension_damping() const { return m_suspension_damping; }
	void set_suspension_damping(float suspension_damping);

	float get_torque() const { return m_torque; }
	void set_torque(float torque);
};

}
