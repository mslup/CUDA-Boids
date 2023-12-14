#include "framework.h"

Shoal::Shoal()
{
	renderParams.height = 0.04f;
	renderParams.width = 0.02f;

	float sqrt3 = glm::sqrt(3);
	float a = renderParams.width;
	float h = renderParams.height;

	std::array<float, vertexCount> vertices = {
		h,		0,				0,
		0, -a / 2, -a * sqrt3 / 6,
		0,	a / 2, -a * sqrt3 / 6,
		0, 		0,  a * sqrt3 / 3
	};
	std::array<float, vertexCount> indices = {
		0, 1, 2,
		0, 2, 3,
		0, 1, 3,
		1, 2, 3
	};

	std::copy(vertices.begin(), vertices.end(), renderParams.vertices);
	std::copy(indices.begin(), indices.end(),   renderParams.indices);

	behaviourParams.sep_factor = (MAX_SEP_FACTOR - MIN_SEP_FACTOR) / 2;
	behaviourParams.aln_factor = (MAX_ALN_FACTOR - MIN_ALN_FACTOR) / 2;
	behaviourParams.coh_factor = (MAX_COH_FACTOR - MIN_COH_FACTOR) / 2;

	behaviourParams.margin = 0.2f;
	behaviourParams.turn_factor = 20;

	behaviourParams.max_speed = 0.9;
	behaviourParams.min_speed = 0.1;

	behaviourParams.visibility_radius = 0.1;

	assert(behaviourParams.visibility_radius >= MIN_R);

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
		positions[i].x = 2 * ((float)(rand()) / (float)(RAND_MAX)) + (-1);
		positions[i].y = 2 * ((float)(rand()) / (float)(RAND_MAX)) + (-1);
		positions[i].z = 0;//2 * ((float)(rand()) / (float)(RAND_MAX)) - 1;
	}

	std::memcpy(positions_bb, positions, Application::N * sizeof(glm::vec3));
}

void Shoal::init_velocities()
{
	for (int i = 0; i < Application::N; i++)
	{
		velocities[i].x = 4 * (float)(rand()) / (float)(RAND_MAX) - 2;
		velocities[i].y = 4 * (float)(rand()) / (float)(RAND_MAX) - 2;
		velocities[i].z = 4 * (float)(rand()) / (float)(RAND_MAX) - 2;
	}

	std::memcpy(velocities_bb, velocities, Application::N * sizeof(glm::vec3));
}