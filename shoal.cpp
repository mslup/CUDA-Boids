#include "framework.h"

void callKernels(int blocks_per_grid, int max_threads,
	double deltaTime, glm::mat4* models, Shoal*, cudaArrays);

glm::mat4 Shoal::calculate_rotate(glm::vec3 pos, glm::vec3 vel)
{
	glm::vec3 v = glm::normalize(vel);

	float c1 = sqrt(v.x * v.x + v.y * v.y);
	float s1 = v.z;

	float c2 = c1 ? v.x / c1 : 1.0;
	float s2 = c1 ? v.y / c1 : 0.0;

	return glm::mat4(
		glm::vec4(v, 0),
		glm::vec4(-s2, c2, 0, 0),
		glm::vec4(-s1 * c2, -s1 * s2, c1, 0),
		glm::vec4(pos, 1)
	);
}


void Shoal::calculate_all_models()
{
	for (int i = 0; i < Application::N; i++)
		models[i] = calculate_rotate(positions[i], velocities[i]);
}

glm::vec3 Shoal::apply_boid_rules(int i, float d)
{
	glm::vec3 separation_component(0);
	glm::vec3 velocity_sum(0);
	glm::vec3 position_sum(0);
	int neighbors = 0;

	for (int j = 0; j < Application::N; ++j)
	{
		glm::vec3 diff = positions[i] - positions[j];
		float len = glm::length(diff);
		glm::vec3 norm = glm::normalize(diff);

		if (i != j && len < behaviourParams.visibility_radius)
		{
			if (len * len > 1e-10)
				separation_component += norm / (len * len);
			else
				separation_component += glm::vec3(0.001, 0.001 * j, 0.001);

			velocity_sum += velocities[j];
			position_sum += positions[j];

			neighbors++;
		}
	}

	glm::vec3 alignment_component;
	glm::vec3 cohesion_component;

	if (neighbors == 0)
		return glm::vec3(0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - velocities[i];
	cohesion_component = position_sum - positions[i];

	return d *
		(behaviourParams.sep_factor / Shoal::SEP_DIVISOR * separation_component
		+ behaviourParams.aln_factor * alignment_component
		+ behaviourParams.coh_factor * cohesion_component);
}

glm::vec3 Shoal::turn_from_wall(glm::vec3 pos, glm::vec3 vel, float d)
{
	float dx_right = -LEFT_WALL - pos.x;
	float dx_left = pos.x - LEFT_WALL;
	float dy_up = -DOWN_WALL - pos.y;
	float dy_down = pos.y - DOWN_WALL;
	float dz_front = -BACK_WALL - pos.z;
	float dz_back = pos.z - BACK_WALL;

	float len = glm::length(vel);

	glm::vec3 vel_change = glm::vec3(0, 0, 0);

	if (dx_right < behaviourParams.margin)
		vel_change.x -= behaviourParams.turn_factor * len * (pos.x + LEFT_WALL + behaviourParams.margin);
	if (dx_left < behaviourParams.margin)
		vel_change.x += behaviourParams.turn_factor * len * (LEFT_WALL - pos.x + behaviourParams.margin);
	if (dy_up < behaviourParams.margin)
		vel_change.y -= behaviourParams.turn_factor * len * (pos.y + DOWN_WALL + behaviourParams.margin);
	if (dy_down < behaviourParams.margin)
		vel_change.y += behaviourParams.turn_factor * len * (DOWN_WALL - pos.y + behaviourParams.margin);
	if (dz_front < behaviourParams.margin)
		vel_change.z -= behaviourParams.turn_factor * len * (pos.z + BACK_WALL + behaviourParams.margin);
	if (dz_back < behaviourParams.margin)
		vel_change.z += behaviourParams.turn_factor * len * (BACK_WALL - pos.z + behaviourParams.margin);

	return vel + d * vel_change;
}

glm::vec3 Shoal::speed_limit(glm::vec3 vel)
{
	if (glm::length(vel) < behaviourParams.min_speed)
		return behaviourParams.min_speed * glm::normalize(vel);
	if (glm::length(vel) > behaviourParams.max_speed)
		return behaviourParams.max_speed * glm::normalize(vel);

	return vel;
}

void Shoal::update_boids_cpu(double d)
{
	for (int i = 0; i < Application::N; ++i)
	{
		positions_bb[i] = glm::vec3(0);
		velocities_bb[i] = glm::vec3(0);

		glm::vec3 new_vel;
		new_vel = velocities[i] + apply_boid_rules(i, (float)d);
		new_vel = turn_from_wall(positions[i], new_vel, (float)d);
		new_vel = speed_limit(new_vel);

		glm::vec3 new_pos = positions[i] + (float)d * new_vel;

		velocities_bb[i] = new_vel;
		positions_bb[i] = new_pos;
		models[i] = calculate_rotate(positions_bb[i], velocities_bb[i]);
	}

	std::memcpy(velocities, velocities_bb, Application::N * sizeof(glm::vec3));
	std::memcpy(positions, positions_bb, Application::N * sizeof(glm::vec3));

	//glBufferData(GL_ARRAY_BUFFER, sizeof(models), &(models)[0], GL_DYNAMIC_DRAW);
}

void Shoal::update_boids_gpu(cudaArrays soa, double d, struct cudaGraphicsResource* cudaVBO)
{
	size_t mat_size = Application::N * sizeof(glm::mat4);
	size_t vec_size = Application::N * sizeof(glm::vec3);
	size_t int_size = Application::N * sizeof(int);
	size_t max_density = (int)glm::ceil(WORLD_WIDTH / Shoal::MIN_GRID_R);
	size_t max_grid_size = max_density * max_density * max_density * sizeof(int);

	gpuErrchk(cudaMemset(soa.grid_boids, 0, int_size));
	gpuErrchk(cudaMemset(soa.grid_cells, 0, int_size));
	gpuErrchk(cudaMemset(soa.grid_starts, -1, max_grid_size));
	gpuErrchk(cudaMemset(soa.grid_ends, -1, max_grid_size));

	const int max_threads = 1024;
	int blocks_per_grid = (Application::N + max_threads - 1) / max_threads;

	glm::mat4* models;
	gpuErrchk(cudaGraphicsMapResources(1, &cudaVBO, 0));
	gpuErrchk(cudaGraphicsResourceGetMappedPointer((void**)&models, NULL, cudaVBO));

	callKernels(blocks_per_grid, max_threads, d, models, this, soa);

	gpuErrchk(cudaGraphicsUnmapResources(1, &cudaVBO, 0));
}