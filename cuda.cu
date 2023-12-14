#include "framework.h"
#include <stdio.h>

__device__ int calculate_grid_index(glm::vec3 pos, float grid_radius)
{
	int gridX = (int)glm::floor((pos.x - LEFT_WALL) / grid_radius);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL) / grid_radius);
	int gridZ = (int)glm::floor((pos.z - BACK_WALL) / grid_radius);

	int gridSize = (int)glm::ceil(WORLD_WIDTH / grid_radius);
	gridX = glm::clamp(gridX, 0, gridSize - 1);
	gridY = glm::clamp(gridY, 0, gridSize - 1);
	gridZ = glm::clamp(gridZ, 0, gridSize - 1);

	return gridZ * gridSize * gridSize
		+ gridY * gridSize
		+ gridX;
}

__device__ void iterate_through_cell(const cudaArrays& soa, const Shoal::behaviourParamsStruct& behaviourParams, int cell, int i, boidParamsStruct boidParams)
{
	glm::vec3* pos = soa.positions;
	glm::vec3* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_ends = soa.grid_ends;
	int* boids = soa.grid_boids;

	int start = grid_starts[cell];

	if (start == -1) // empty cell
		return;

	int end = grid_ends[cell];

	for (int k = start; k < end; k++)
	{
		int j = boids[k];
		if (i == j) continue;

		glm::vec3 diff = pos[i] - pos[j];
		float len = glm::length(diff);
		glm::vec3 norm = glm::normalize(diff);

		if (len < behaviourParams.visibility_radius)
		{
			if (len * len > 1e-10)
				(*boidParams.separation_component) += norm / (len * len);
			else
				(*boidParams.separation_component) += glm::vec3(0.001, 0.001 * j, 0.001);

			(*boidParams.velocity_sum) += vel[j];
			(*boidParams.position_sum) += pos[j];
			(*boidParams.neighbors)++;
		}
	}
}

__device__ glm::vec3 apply_boid_rules(const cudaArrays &soa, const Shoal::behaviourParamsStruct& behaviourParams, int i, float d)
{
	glm::vec3 separation_component(0);
	glm::vec3 velocity_sum(0);
	glm::vec3 position_sum(0);
	int neighbors = 0;

	boidParamsStruct boidParams;

	boidParams.separation_component = &separation_component;
	boidParams.velocity_sum = &velocity_sum;
	boidParams.position_sum = &position_sum;
	boidParams.neighbors = &neighbors;

#ifdef NAIVE
	for (int j = 0; j < Application::N; ++j)
	{
		glm::vec3 diff = soa.positions[i] - soa.positions[j];
		float len = glm::length(diff);
		glm::vec3 norm = glm::normalize(diff);

		if (i != j && len < params.visibility_radius)
		{
			separation_component += norm / len;
			velocity_sum += soa.velocities[j];
			position_sum += soa.positions[j];

			neighbors++;
		}
	}
#else
	float grid_radius = 2 * behaviourParams.visibility_radius;
	int density = (int)glm::ceil(WORLD_WIDTH / grid_radius);
	int cell = calculate_grid_index(soa.positions[i], grid_radius);
	int gridX = cell % density;
	int gridY = (cell / density) % density;
	int gridZ = cell / (density * density);

	int densitysq = density * density;
	int grid_size = density * densitysq;

	int x_offset, y_offset, z_offset;

	if (soa.positions[i].x >= LEFT_WALL + (gridX + 0.5) * grid_radius)
		x_offset = 1;
	else
		x_offset = -1;

	if (soa.positions[i].y >= DOWN_WALL + (gridY + 0.5) * grid_radius)
		y_offset = density;
	else
		y_offset = -density;

	if (soa.positions[i].z >= BACK_WALL + (gridZ + 0.5) * grid_radius)
		z_offset = densitysq;
	else
		z_offset = -densitysq;

	int new_cell;
	for (int k = 0; k < 2; k++) // in two grid planes
	{
		new_cell = cell + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size)
			iterate_through_cell(soa, behaviourParams, new_cell, i, boidParams);

		new_cell = cell + x_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / density == (new_cell - x_offset) / density) // stay in the same x-row
			iterate_through_cell(soa, behaviourParams, new_cell, i, boidParams);

		new_cell = cell + y_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / densitysq == (new_cell - y_offset) / densitysq) // stay in the same y-row (column)
			iterate_through_cell(soa, behaviourParams, new_cell, i, boidParams);

		new_cell = cell + x_offset + y_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / density == (new_cell - x_offset) / density // stay in the same x-row
			&& new_cell / densitysq == (new_cell - y_offset) / densitysq) // stay in the same y-row (column)
			iterate_through_cell(soa, behaviourParams, new_cell, i, boidParams);
	}
#endif

	glm::vec3 alignment_component;
	glm::vec3 cohesion_component;

	if (neighbors == 0)
		return glm::vec3(0, 0, 0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - soa.velocities[i];
	cohesion_component = position_sum - soa.positions[i];

	return d * glm::vec3
		(behaviourParams.sep_factor / Shoal::SEP_DIVISOR * separation_component
		+ behaviourParams.aln_factor * alignment_component
		+ behaviourParams.coh_factor * cohesion_component);
}

__device__ glm::vec3 speed_limit(glm::vec3 vel, const Shoal::behaviourParamsStruct& behaviourParams)
{
	if (glm::length(vel) < behaviourParams.min_speed)
		return behaviourParams.min_speed * glm::normalize(vel);
	if (glm::length(vel) > behaviourParams.max_speed)
		return behaviourParams.max_speed * glm::normalize(vel);

	return vel;
}

__device__ glm::vec3 turn_from_wall(glm::vec3 pos, glm::vec3 vel, const Shoal::behaviourParamsStruct& behaviourParams, float d)
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

__global__ void calculateGridKernel(const cudaArrays soa, float visibility_radius)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	soa.grid_cells[i] = calculate_grid_index(soa.positions[i], 2 * visibility_radius);
	soa.grid_boids[i] = i;
}

__global__ void calculateGridStartsKernel(const cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	int cell = soa.grid_cells[i];
	int prev_cell = i != 0 ? soa.grid_cells[i - 1] : -1;

	if (i == 0 || cell != prev_cell)
	{
		soa.grid_starts[cell] = i;
		if (i != 0)
			soa.grid_ends[prev_cell] = i;
		else
			soa.grid_ends[soa.grid_cells[Application::N - 1]] = Application::N;
	}
}

__global__ void calculateBoidsKernel(const cudaArrays soa,
	Shoal::behaviourParamsStruct behaviourParams,
	double d, glm::mat4* models)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	soa.positions_bb[i] = glm::vec3(0);
	soa.velocities_bb[i] = glm::vec3(0);

	glm::vec3 new_vel;
	new_vel = soa.velocities[i] + apply_boid_rules(soa, behaviourParams, i, (float)d);
	new_vel = turn_from_wall(soa.positions[i], new_vel, behaviourParams, (float)d);
	new_vel = speed_limit(new_vel, behaviourParams);

	glm::vec3 new_pos = soa.positions[i] + (float)d * new_vel;

	soa.velocities_bb[i] = new_vel;
	soa.positions_bb[i] = new_pos;
	
	__syncthreads();

	soa.positions[i] = soa.positions_bb[i];
	soa.velocities[i] = soa.velocities_bb[i];

	glm::vec3 v = glm::normalize(soa.velocities[i]);

	float c1 = sqrt(v.x * v.x + v.y * v.y);
	float s1 = v.z;

	float c2 = c1 ? v.x / c1 : 1.0;
	float s2 = c1 ? v.y / c1 : 0.0;

	models[i] = glm::mat4(
		glm::vec4(v, 0),
		glm::vec4(-s2, c2, 0, 0),
		glm::vec4(-s1 * c2, -s1 * s2, c1, 0),
		glm::vec4(soa.positions[i], 1)
	);
}