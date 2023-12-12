#include "framework.h"

Shoal::Shoal()
{
	params.height = 0.04f;
	params.width = 0.02f;

	/*
	params.vertices[0] = params.height / 2;
	params.vertices[1] = 0;
	params.vertices[2] = -params.height / 2;
	params.vertices[3] = -params.width / 2;
	params.vertices[4] = -params.height / 2;
	params.vertices[5] = params.width / 2;
	*/


	float sqrt3 = glm::sqrt(3);
	float a = params.width;

	/*
	// A
	params.vertices[0] = params.height;
	params.vertices[1] = 0;
	params.vertices[2] = 0;
	// B
	params.vertices[3] = 0;
	params.vertices[4] = -a/ 2;
	params.vertices[5] = -a * sqrt3 / 6;
	// C
	params.vertices[6] = 0;
	params.vertices[7] = a / 2;
	params.vertices[8] = -a * sqrt3 / 6;
	// D
	params.vertices[9] = 0;
	params.vertices[10] = 0;
	params.vertices[11] = a * sqrt3 / 3;
	*/

	std::array<float, vertexCount> vertices = {
		params.height,		0,				0,
		0,			   -a / 2, -a * sqrt3 / 6,
		0,				a / 2, -a * sqrt3 / 6,
		0,					0,  a * sqrt3 / 3
	};

	std::array<float, vertexCount> indices = {
		0, 1, 2,
		0, 2, 3,
		0, 1, 3,
		1, 2, 3
	};

	std::copy(vertices.begin(), vertices.end(), params.vertices);
	std::copy(indices.begin(), indices.end(), params.indices);

	params.s = 0.001;
	params.a = 0.1;
	params.c = 0.005;

	params.margin = 0.2f;
	params.turn = 5e-4;

	params.max_speed = 0.9;
	params.min_speed = 0.1;

	params.visibility_radius = 1e-1;

	init_positions();
	init_velocities();

	for (int i = 0; i < Application::N; i++)
	{
		models[i] = calculate_rotate(positions[i], velocities[i]);
	}
}

void Shoal::init_positions()
{
	for (int i = 0; i < Application::N; i++)
	{
		positions[i].x = 2 * ((float)(rand()) / (float)(RAND_MAX)-0.5);
		positions[i].y = 2 * ((float)(rand()) / (float)(RAND_MAX)-0.5);
		positions[i].z = 2 * ((float)(rand()) / (float)(RAND_MAX)-0.5);
	}

	std::memcpy(positions_bb, positions, Application::N * sizeof(glm::vec3));
}

void Shoal::init_velocities()
{
	for (int i = 0; i < Application::N; i++)
	{
		velocities[i].x = ((float)(rand()) / (float)(RAND_MAX)-0.5);
		velocities[i].y = ((float)(rand()) / (float)(RAND_MAX)-0.5);
		velocities[i].z = 0;
	}

	std::memcpy(velocities_bb, velocities, Application::N * sizeof(glm::vec3));
}