#pragma once
#include "framework.h"

class cpu_shoal
{
public:
	float s, a, c, margin, turn, max_speed, min_speed, visibility_radius;
	float height, width;
	float vertices[6];

	glm::vec2 positions[N];
	glm::vec2 positions_bb[N];
	glm::vec2 velocities[N];
	glm::vec2 velocities_bb[N];
	glm::mat3 model[N];

	cpu_shoal();

	void init_positions();
	void init_velocities();

	glm::mat3 calculate_rotate(glm::vec2 pos, glm::vec2 vel);

	void update_boids(double d);
	void calculate_all_models();

private:
	void apply_boid_rules(int i);

	void turn_from_wall(int i);

	void speed_limit(int i);

	void teleport_through_wall(int i);

};

