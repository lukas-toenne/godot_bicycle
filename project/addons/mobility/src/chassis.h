#pragma once

#include "Godot.hpp"
#include "PhysicsDirectBodyState.hpp"
#include "RigidBody.hpp"

#include <unordered_map>
#include <vector>

// Custom specialization of std::hash for RID.
template<>
struct std::hash<godot::RID>
{
	std::size_t operator()(godot::RID const& rid) const noexcept
	{
		return std::hash<int32_t>{}(rid.get_id());
	}
};

namespace godot {

class Wheel;

class Chassis : public RigidBody {
	GODOT_CLASS(Chassis, RigidBody)

public:
	struct WheelInfo
	{
		Wheel* wheel;

		Transform rest_transform;

		// Contact vectors in world space
		Vector3 hard_point;
		Vector3 wheel_direction;
		Vector3 wheel_axle;
		Vector3 contact_normal;
		Vector3 contact_point;

		float suspension_length;
		bool is_in_contact;
		CollisionObject* ground_object;
	};

	struct WheelConstraint
	{

	};

	struct BodySolution
	{
		Vector3 delta_vel = Vector3(0, 0, 0);
		Vector3 delta_rot = Vector3(0, 0, 0);
	};

	static void _register_methods();

	Chassis();
	~Chassis();

	void _init(); // our initializer called by Godot

	void _integrate_forces(PhysicsDirectBodyState* state);

	void add_wheel(Wheel* wheel);
	void remove_wheel(Wheel* wheel);

protected:
	void ray_cast_wheel(PhysicsDirectBodyState* state, WheelInfo& info);
	void solve_constraints(PhysicsDirectBodyState* state);
	float resolve_single_constraint(WheelConstraint& constraint);

private:
	std::vector<WheelInfo> m_wheels;
	std::vector<WheelConstraint> m_constraints;
	std::unordered_map<RID, BodySolution> m_body_solutions;

	static const Chassis m_cdo;
};

}
