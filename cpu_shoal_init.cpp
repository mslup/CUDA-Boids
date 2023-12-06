#include "framework.h"

cpu_shoal::cpu_shoal()
	: height{ 0.04f }, width{ 0.02f }, 
	vertices{ height / 2, 0, -height / 2, -width / 2, -height / 2, width / 2 }
{
	s = 0.01;//3e-2;
	a = 0.1;//8e-2;
	c = 0.01;//9e-2;

	margin = 0.2f;
	turn = 5e-3;

	max_speed = 1e-2;
	min_speed = 9e-3;

	visibility_radius = 1e-1;

	/*glm::vec2 center(0, 0);
	vertices[0] = center.x + height / 2, center.y;
	vertices[1] = center.x - height / 2, center.y - width / 2;
	vertices[2] = center.x - height / 2, center.y + width / 2;*/

	init_positions();
	init_velocities();
	for (int i = 0; i < N; i++)
	{
		model[i] = calculate_rotate(positions[i], velocities[i]);
	}
}

void cpu_shoal::init_positions()
{
	for (int i = 0; i < N; i++)
	{
		positions[i].x = (float)(rand()) / (float)(RAND_MAX)-0.5;
		positions[i].y = (float)(rand()) / (float)(RAND_MAX)-0.5;
	}

	std::memcpy(positions_bb, positions, N * sizeof(glm::vec2));
}

void cpu_shoal::init_velocities()
{
	for (int i = 0; i < N; i++)
	{
		velocities[i].x = ((float)(rand()) / (float)(RAND_MAX)-0.5);
		velocities[i].y = ((float)(rand()) / (float)(RAND_MAX)-0.5);
	}

	std::memcpy(velocities_bb, velocities, N * sizeof(glm::vec2));
}