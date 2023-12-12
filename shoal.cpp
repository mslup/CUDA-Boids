#include "framework.h"

glm::mat4 Shoal::calculate_rotate(glm::vec3 pos, glm::vec3 vel)
{
	//glm::vec3 v = glm::normalize(vel);
	//glm::vec3 vT = glm::vec3(v.y, -v.x, 0); // LUB 1 ???????????
	//return glm::mat4(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(pos, 1.0f));
	return glm::mat4(1.0);
}

void Shoal::update_boids_cpu(double d)
{
	for (int i = 0; i < Application::N; ++i)
	{
		apply_boid_rules(i);
		turn_from_wall(i);
		speed_limit(i);
		positions_bb[i] += (float)d * velocities_bb[i];
		teleport_through_wall(i);

		models[i] = calculate_rotate(positions_bb[i], velocities_bb[i]);
	}

	std::memcpy(velocities, velocities_bb, Application::N * sizeof(glm::vec3));
	std::memcpy(positions, positions_bb, Application::N * sizeof(glm::vec3));

	glBufferData(GL_ARRAY_BUFFER, sizeof(models), &(models)[0], GL_DYNAMIC_DRAW);
}

void Shoal::calculate_all_models()
{
	for (int i = 0; i < Application::N; i++)
		models[i] = calculate_rotate(positions[i], velocities[i]);
}

void Shoal::apply_boid_rules(int i)
{
	glm::vec3 separation_component(0);
	glm::vec3 velocity_sum(0);
	glm::vec3 position_sum(0);
	int neighbors = 0;

	for (int j = 0; j < Application::N; ++j)
	{
		float len = glm::length(positions[i] - positions[j]);
		if (i != j && len < params.visibility_radius)
		{
			separation_component += positions[i] - positions[j];
			velocity_sum += velocities[j];
			position_sum += positions[j];

			neighbors++;
		}
	}

	glm::vec3 alignment_component;
	glm::vec3 cohesion_component;

	if (neighbors == 0)
		return;

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - velocities[i];
	cohesion_component = position_sum - positions[i];

	velocities_bb[i] += params.s * separation_component 
		+ params.a * alignment_component 
		+ params.c * cohesion_component;
}

void Shoal::turn_from_wall(int i)
{
	float dx_right = 1 - positions_bb[i].x;
	float dx_left = positions_bb[i].x + 1;
	float dy_up = 1 - positions_bb[i].y;
	float dy_down = positions_bb[i].y + 1;

	float len = glm::length(velocities_bb[i]);

	if (dx_right < params.margin)
		velocities_bb[i].x -= params.turn * len / (dx_right * dx_right);
	if (dx_left < params.margin)
		velocities_bb[i].x += params.turn * len / (dx_left * dx_left);
	if (dy_up < params.margin)
		velocities_bb[i].y -= params.turn * len / (dy_up * dy_up);
	if (dy_down < params.margin)
		velocities_bb[i].y += params.turn * len / (dy_down * dy_down);
}

void Shoal::speed_limit(int i)
{
	if (glm::length(velocities_bb[i]) < params.min_speed)
		velocities_bb[i] = params.min_speed * glm::normalize(velocities_bb[i]);
	if (glm::length(velocities_bb[i]) > params.max_speed)
		velocities_bb[i] = params.max_speed * glm::normalize(velocities_bb[i]);
}

void Shoal::teleport_through_wall(int i)
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

void Shoal::update_boids_gpu(cudaArrays soa, double d, struct cudaGraphicsResource* cudaVBO)
{
	size_t mat_size = Application::N * sizeof(glm::mat4);
	size_t vec_size = Application::N * sizeof(glm::vec3);
	size_t int_size = Application::N * sizeof(int);
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	cudaMemset(soa.grid_boids, 0, int_size);
	cudaMemset(soa.grid_cells, 0, int_size);
	cudaMemset(soa.grid_starts, -1, grid_size);
	cudaMemset(soa.grid_ends, -1, grid_size);

	const int max_threads = 1024;
	int blocks_per_grid = (Application::N + max_threads - 1) / max_threads;

	glm::mat4* models;
	cudaGraphicsMapResources(1, &cudaVBO, 0);
	cudaGraphicsResourceGetMappedPointer((void**)&models, NULL, cudaVBO);

	callKernels(blocks_per_grid, max_threads, d, models, this, soa);

	cudaGraphicsUnmapResources(1, &cudaVBO, 0);
}