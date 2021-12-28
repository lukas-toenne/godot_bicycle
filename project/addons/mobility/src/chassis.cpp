#include "chassis.h"

#include "wheel.h"

#include "PhysicsDirectSpaceState.hpp"
#include "World.hpp"

using namespace godot;

const Chassis Chassis::m_cdo = Chassis();

void Chassis::_register_methods() {
	register_signal<Chassis>("wheels_changed");

	register_method("_integrate_forces", &Chassis::_integrate_forces);
}

Chassis::Chassis() {
}

Chassis::~Chassis() {
}

void Chassis::_init() {
}

void Chassis::_integrate_forces(PhysicsDirectBodyState* state)
{
}

void Chassis::solve_constraints(PhysicsDirectBodyState* state)
{
	const float least_squares_residual_threshold = 0.001f;

	m_body_solutions.clear();
	
	int max_iterations = 50;
	for (int iteration = 0; iteration < max_iterations; ++iteration)
	{
		//	print("  iteration ", iteration)
		float least_squares_residual = 0.0;
		for (WheelConstraint& constraint : m_constraints)
		{
			float residual = resolve_single_constraint(constraint);
			least_squares_residual = std::max(least_squares_residual, residual * residual);
			//print("wheel ", wheel_array[k], " residual=", residual)
			//print("  wheel ", wheel, " total lambda", total_lambda[k])
		}

		//DebugEventRecorder.record_vector(self, "impulse" + str(wheel), state.transform.origin, state.angular_velocity)
		if (least_squares_residual <= least_squares_residual_threshold)
		{
			//print("%d iterations, residual=%f" % [iteration + 1, least_squares_residual])
			break;
		}

		if (iteration == max_iterations + 1)
		{
			//print("MAX ITERATIONS REACHED")
		}

		auto iter = m_body_solutions.find(get_rid());
		if (iter != m_body_solutions.end())
		{
			BodySolution& solA = iter->second;
			state->set_linear_velocity(state->get_linear_velocity() + solA.delta_vel);
			state->set_angular_velocity(state->get_angular_velocity() + solA.delta_rot);
			//DebugEventRecorder.record_vector(self, "linear_impulse", state.transform.origin, body_solutions[get_rid()].delta_vel)
			//DebugEventRecorder.record_vector(self, "angular_impulse", state.transform.origin, body_solutions[get_rid()].delta_rot)
		}
	}
}

float Chassis::resolve_single_constraint(WheelConstraint& constraint)
{
	return 0.0f;
}

void Chassis::add_wheel(Wheel* wheel)
{
	WheelInfo info;
	info.wheel = wheel;
	info.rest_transform = wheel->get_transform();
	m_wheels.emplace_back(info);

	emit_signal("wheels_changed");
}

void Chassis::remove_wheel(Wheel* wheel)
{
	std::vector<WheelInfo> new_wheels;
	new_wheels.reserve(m_wheels.size());
	for (auto iter = m_wheels.begin(); iter != m_wheels.end(); ++iter)
	{
		if (iter->wheel != wheel)
		{
			new_wheels.push_back(*iter);
		}
	}

	if (new_wheels.size() < m_wheels.size())
	{
		emit_signal("wheels_changed");
	}

	m_wheels = new_wheels;
}

void Chassis::ray_cast_wheel(PhysicsDirectBodyState* state, Chassis::WheelInfo& info)
{
	PhysicsDirectSpaceState* space_state = get_world()->get_direct_space_state();
	const Transform chassis_transform = state->get_transform();

	Transform wheel_transform = chassis_transform * info.rest_transform;

	info.hard_point = wheel_transform.origin;
	info.wheel_direction = -wheel_transform.basis.y.normalized();
	info.wheel_axle = wheel_transform.basis.x.normalized();

	// TODO For accurate future collision detection use cast_motion :
	// space_state.cast_motion()
	Vector3 source = info.hard_point - info.wheel_direction * info.wheel->m_radius;
	Vector3 target = info.hard_point + info.wheel_direction * (info.wheel->m_radius + info.wheel->m_suspension_rest_length);
	//DebugEventRecorder.record_vector(wheel, "raycast", source, target - source)
	Array exclude;
	exclude.push_back(this);
	Dictionary result = space_state->intersect_ray(source, target, exclude, get_collision_mask());

	if (result.empty())
	{
		info.is_in_contact = false;
		info.ground_object = nullptr;
		info.contact_point = target;
		info.contact_normal = -info.wheel_direction;
		info.suspension_length = info.wheel->m_suspension_rest_length;
	}
	else
	{
		info.is_in_contact = true;
		info.ground_object = result["collider"];
		info.contact_point = result["position"];
		info.contact_normal = result["normal"];

		float distance = source.distance_to(info.contact_point);
		// float param = distance / (2.0 * wheel.radius + wheel.rest_length);
		//info.suspension_length = clamp(distance - 2.0 * info.wheel->m_radius, info.wheel->m_suspension_rest_length - info.wheel->m, wheel.rest_length + wheel.travel);

		//DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.relative_contact_velocity)
		//DebugEventRecorder.record_vector(wheel, "contact_velocity", info.contact_point, info.contact_normal * info.suspension_relative_velocity)

		if (info.is_in_contact)
		{
			//DebugEventRecorder.record_spring(wheel, "suspension", info.hard_point, info.wheel_direction, wheel.rest_length, info.suspension_length)
			//	#			print("Wheel {wheel} in contact: ground={ground}, point={point}, normal={normal}, suspension={suslen}, rel.vel.={relvel}, proj.suspension={invproj}".format({
			//	#				"wheel":wheel,
			//	#				"ground":info.ground_object,
			//	#				"point":info.contact_point,
			//	#				"normal":info.contact_normal,
			//	#				"suslen":info.suspension_length,
			//	#				"relvel":info.suspension_relative_velocity,
			//	#				"invproj":info.clipped_inv_contact_dot_suspension,
			//	#				}))
		}
		else
		{
			//DebugEventRecorder.clear_event(wheel, "suspension")
			//	#			print("Wheel {wheel} free [ground={ground}, point={point}, normal={normal}, suspension={suslen}, rel.vel.={relvel}, proj.suspension={invproj}]".format({
			//	#				"wheel":wheel,
			//	#				"ground":info.ground_object,
			//	#				"point":info.contact_point,
			//	#				"normal":info.contact_normal,
			//	#				"suslen":info.suspension_length,
			//	#				"relvel":info.suspension_relative_velocity,
			//	#				"invproj":info.clipped_inv_contact_dot_suspension,
			//	#				}))
		}
	}
}