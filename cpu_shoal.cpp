#include "framework.h"

glm::mat3 cpu_shoal::calculate_rotate(glm::vec2 pos, glm::vec2 vel)
{
	glm::vec2 v = glm::normalize(vel);
	glm::vec2 vT = glm::vec2(v.y, -v.x);
	return glm::mat3(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(pos, 1.0f));
}

void cpu_shoal::update_boids()
{
	for (int i = 0; i < N; ++i)
	{
		apply_boid_rules(i);
		turn_from_wall(i);
		speed_limit(i);
		positions_bb[i] += velocities_bb[i];
		teleport_through_wall(i);

		model[i] = calculate_rotate(positions_bb[i], velocities_bb[i]);
	}

	std::memcpy(velocities, velocities_bb, N * sizeof(glm::vec2));
	std::memcpy(positions, positions_bb, N * sizeof(glm::vec2));
}

void cpu_shoal::calculate_all_models()
{
	for (int i = 0; i < N; i++)
		model[i] = calculate_rotate(positions[i], velocities[i]);
}

void cpu_shoal::apply_boid_rules(int i)
{
	glm::vec2 separation_component(0, 0);
	glm::vec2 velocity_sum(0, 0);
	glm::vec2 position_sum(0, 0);
	int neighbors = 0;

	for (int j = 0; j < N; ++j)
	{
		float len = glm::length(positions[i] - positions[j]);
		if (i != j && len < visibility_radius)
		{
			separation_component += positions[i] - positions[j];
			velocity_sum += velocities[j];
			position_sum += positions[j];

			neighbors++;
		}
	}

	glm::vec2 alignment_component;
	glm::vec2 cohesion_component;

	if (neighbors == 0)
		return;

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - velocities[i];
	cohesion_component = position_sum - positions[i];

	velocities_bb[i] += s * separation_component + a * alignment_component + c * cohesion_component;
}

void cpu_shoal::turn_from_wall(int i)
{
	float dx_right = 1 - positions_bb[i].x;
	float dx_left = positions_bb[i].x + 1;
	float dy_up = 1 - positions_bb[i].y;
	float dy_down = positions_bb[i].y + 1;

	float len = glm::length(velocities_bb[i]);

	if (dx_right < margin)
		velocities_bb[i].x -= turn * len / (dx_right * dx_right);
	if (dx_left < margin)
		velocities_bb[i].x += turn * len / (dx_left * dx_left);
	if (dy_up < margin)
		velocities_bb[i].y -= turn * len / (dy_up * dy_up);
	if (dy_down < margin)
		velocities_bb[i].y += turn * len / (dy_down * dy_down);
}

void cpu_shoal::speed_limit(int i)
{
	if (glm::length(velocities_bb[i]) < min_speed)
		velocities_bb[i] = min_speed * glm::normalize(velocities_bb[i]);
	if (glm::length(velocities_bb[i]) > max_speed)
		velocities_bb[i] = max_speed * glm::normalize(velocities_bb[i]);
}

void cpu_shoal::teleport_through_wall(int i)
{
	if (positions_bb[i].x > 1)
		positions_bb[i].x = -1;
	if (positions_bb[i].y > 1)
		positions_bb[i].y = -1;
	if (positions_bb[i].x < -1)
		positions_bb[i].x = 1;
	if (positions_bb[i].y < -1)
		positions_bb[i].y = 1;
}