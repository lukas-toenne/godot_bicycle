#pragma once

#include <godot_cpp/classes/physics_direct_body_state3d.hpp>
#include <godot_cpp/classes/rigid_dynamic_body3d.hpp>

#include <unordered_map>
#include <vector>

// Custom specialization of std::hash for RID.
template<>
struct std::hash<godot::RID>
{
	std::size_t operator()(godot::RID const& rid) const noexcept
	{
		return std::hash<int64_t>{}(rid.get_id());
	}
};

namespace godot {

class Wheel;

class Chassis : public RigidDynamicBody3D {
	GDCLASS(Chassis, RigidDynamicBody3D)

public:
	struct WheelInfo
	{
		Wheel* wheel;

		Transform3D rest_transform;

		// Contact vectors in world space
		Vector3 hard_point;
		Vector3 wheel_direction;
		Vector3 wheel_axle;
		Vector3 contact_normal;
		Vector3 contact_point;

		float suspension_length;
		bool is_in_contact;
		CollisionObject3D* ground_object;
	};

	struct WheelConstraint
	{

	};

	struct BodySolution
	{
		Vector3 delta_vel = Vector3(0, 0, 0);
		Vector3 delta_rot = Vector3(0, 0, 0);
	};

	static void _bind_methods();

	Chassis();
	~Chassis();

	virtual void _integrate_forces(PhysicsDirectBodyState3D* state) override;

	void add_wheel(Wheel* wheel);
	void remove_wheel(Wheel* wheel);

protected:
	void ray_cast_wheel(PhysicsDirectBodyState3D* state, WheelInfo& info);
	void solve_constraints(PhysicsDirectBodyState3D* state);
	float resolve_single_constraint(WheelConstraint& constraint);

private:
	std::vector<WheelInfo> m_wheels;
	std::vector<WheelConstraint> m_constraints;
	std::unordered_map<RID, BodySolution> m_body_solutions;
};

}
