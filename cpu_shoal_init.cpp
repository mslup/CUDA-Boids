#include "framework.h"

cpu_shoal::cpu_shoal()
{
	params.height = 0.04f;
	params.width = 0.02f;

	params.vertices[0] = params.height / 2;
	params.vertices[1] = 0;
	params.vertices[2] = -params.height / 2;
	params.vertices[3] = -params.width / 2;
	params.vertices[4] = -params.height / 2;
	params.vertices[5] = params.width / 2;

	params.s = 0.01;
	params.a = 0.1;
	params.c = 0.005;

	params.margin = 0.2f;
	params.turn = 5e-4;

	params.max_speed = 0.9;
	params.min_speed = 0.1;

	params.visibility_radius = 1e-1;

	init_positions();
	init_velocities();
	for (int i = 0; i < N; i++)
	{
		models[i] = calculate_rotate(positions[i], velocities[i]);
	}
}

void cpu_shoal::init_positions()
{
	float offset = 2.0f / glm::sqrt(N);

	for (int i = 0; i < N; i++)
	{
		positions[i].x = 2 * ((float)(rand()) / (float)(RAND_MAX)-0.5);
		positions[i].y = 2 * ((float)(rand()) / (float)(RAND_MAX)-0.5);
	}

	//for (int i = 0; i < 8; i++)
	//	for (int j = 0; j < 8; j++)
	//	{
	//		positions[8 * i + j].x = -1 + i * offset;
	//		positions[8 * i + j].y = -1 + j * offset;
	//	}

	std::memcpy(positions_bb, positions, N * sizeof(glm::vec2));
}

void cpu_shoal::init_velocities()
{
	for (int i = 0; i < N; i++)
	{
		velocities[i].x = 0;//((float)(rand()) / (float)(RAND_MAX)-0.5);
		velocities[i].y = 1;//((float)(rand()) / (float)(RAND_MAX)-0.5);
	}

	std::memcpy(velocities_bb, velocities, N * sizeof(glm::vec2));
}