#include "framework.h"
#include <stdio.h>

//#define NAIVE
//#define SNAP_TO_GRID

// todo: add declarations in cuh; reclamp 
__device__ int calculate_grid_index(glm::vec3 pos, float grid_radius)
{
	int gridX = (int)glm::floor((pos.x - LEFT_WALL) / grid_radius);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL) / grid_radius);
	int gridZ = (int)glm::floor((pos.z - BACK_WALL) / grid_radius);

	//printf("(%f - %f) / %f = %f | %d\n", pos.x, LEFT_WALL, MIN_GRID_R, (pos.x - LEFT_WALL) / MIN_GRID_R, gridX);

	int gridSize = (int)glm::ceil(WORLD_WIDTH / grid_radius);
	gridX = glm::clamp(gridX, 0, gridSize - 1);
	gridY = glm::clamp(gridY, 0, gridSize - 1);
	gridZ = glm::clamp(gridZ, 0, gridSize - 1);

	return gridZ * gridSize * gridSize
		+ gridY * gridSize
		+ gridX;
}

struct boidParamsStruct {
	glm::vec3* separation_component;
	glm::vec3* velocity_sum;
	glm::vec3* position_sum;
	int* neighbors;
} boidParams;

// todo: everything to const ref
__device__ void iterate_through_cell(const cudaArrays& soa, const Shoal::paramsStruct &params, int cell, int i, boidParamsStruct boidParams)
{
	glm::vec3* pos = soa.positions;
	glm::vec3* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_ends = soa.grid_ends;
	int* boids = soa.grid_boids;

	// todo: merge R with visib_radius
	//float radius_sq = params.visibility_radius * params.visibility_radius;

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

		if (len < params.visibility_radius)
		{
			//soa.velocities_bb[j] = glm::vec3(0, 1, 0);

			(*boidParams.separation_component) += norm / (len * len);
			(*boidParams.velocity_sum) += vel[j];
			(*boidParams.position_sum) += pos[j];
			(*boidParams.neighbors)++;
		}
	}
}

__device__ glm::vec3 apply_boid_rules(cudaArrays soa, const Shoal::paramsStruct& params, int i, double d)
{
	glm::vec3 separation_component(0);
	glm::vec3 velocity_sum(0);
	glm::vec3 position_sum(0);
	int neighbors = 0;
	//float radius_sq = params.visibility_radius * params.visibility_radius;

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
	float grid_radius = 2 * params.visibility_radius;
	int density = (int)glm::ceil(WORLD_WIDTH / grid_radius);
	int cell = calculate_grid_index(soa.positions[i], grid_radius);
	int gridX = cell % density;
	int gridY = (cell / density) % density;
	int gridZ = cell / (density * density);

	int densitysq = density * density;
	int grid_size = density * densitysq;

	// todo: calculate this beforehand? sort a duplicate buffer?

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
			iterate_through_cell(soa, params, new_cell, i, boidParams);

		new_cell = cell + x_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / density == (new_cell - x_offset) / density) // stay in the same x-row
			iterate_through_cell(soa, params, new_cell, i, boidParams);

		new_cell = cell + y_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / densitysq == (new_cell - y_offset) / densitysq) // stay in the same y-row (column)
			iterate_through_cell(soa, params, new_cell, i, boidParams);

		new_cell = cell + x_offset + y_offset + k * z_offset;
		if (new_cell >= 0 && new_cell < grid_size
			&& new_cell / density == (new_cell - x_offset) / density // stay in the same x-row
			&& new_cell / densitysq == (new_cell - y_offset) / densitysq) // stay in the same y-row (column)
			iterate_through_cell(soa, params, new_cell, i, boidParams);
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

	/*printf("cohesion: %f %f %f %f\n",
		params.c,
		cohesion_component.x,
		cohesion_component.y,
		cohesion_component.z);*/

	glm::vec3 ret =
		(params.s * separation_component
			+ params.a * alignment_component
			+ params.c * cohesion_component);

	//printf("ret in function %f %f %f\n", ret.x, ret.y, ret.z);

	return ret;
}

__device__ glm::vec3 speed_limit(glm::vec3 vel, const Shoal::paramsStruct& params)
{
	if (glm::length(vel) < params.min_speed)
		return params.min_speed * glm::normalize(vel);
	if (glm::length(vel) > params.max_speed)
		return params.max_speed * glm::normalize(vel);

	return vel;
}

__device__ glm::vec3 turn_from_wall(glm::vec3 pos, glm::vec3 vel, const Shoal::paramsStruct& params)
{
	float dx_right = 1 - pos.x;
	float dx_left = pos.x + 1;
	float dy_up = 1 - pos.y;
	float dy_down = pos.y + 1;
	float dz_front = 1 - pos.z;
	float dz_back = pos.z + 1;

	float len = glm::length(vel);

	glm::vec3 vel_change = glm::vec3(0, 0, 0);

	if (dx_right < params.margin)
		vel_change.x -= params.turn * len / (dx_right * dx_right);
	if (dx_left < params.margin)
		vel_change.x += params.turn * len / (dx_left * dx_left);
	if (dy_up < params.margin)
		vel_change.y -= params.turn * len / (dy_up * dy_up);
	if (dy_down < params.margin)
		vel_change.y += params.turn * len / (dy_down * dy_down);
	if (dz_front < params.margin)
		vel_change.z -= params.turn * len / (dz_front * dz_front);
	if (dz_back < params.margin)
		vel_change.z += params.turn * len / (dz_back * dz_back);

	return vel + vel_change;
}

__device__ glm::vec3 teleport_through_wall(glm::vec3 pos)
{
	glm::vec3 ret = pos;

	if (pos.x > 1)
		ret.x = -1;
	if (pos.y > 1)
		ret.y = -1;
	if (pos.z > 1)
		ret.z = -1;

	if (pos.x < -1)
		ret.x = 1;
	if (pos.y < -1)
		ret.y = 1;
	if (pos.z < -1)
		ret.z = 1;

	return ret;
}

__global__ void calculateGridKernel(struct cudaArrays soa, float visibility_radius)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	//size_t density = (int)glm::ceil(WORLD_WIDTH / visibility_radius);
	//size_t grid_size = density * density * density * sizeof(int);

	soa.grid_cells[i] = calculate_grid_index(soa.positions[i], 2 * visibility_radius);
	soa.grid_boids[i] = i;
}

__global__ void calculateGridStartsKernel(struct cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;


	//size_t density = (int)glm::ceil(WORLD_WIDTH / MIN_GRID_R);
	//size_t grid_size = density * density * density * sizeof(int);

	int cell = soa.grid_cells[i];
	int prev_cell = i != 0 ? soa.grid_cells[i - 1] : -1;

	if (i == 0 || cell != prev_cell)
	{
		// todo: there's a bottleneck
		soa.grid_starts[cell] = i;
		if (i != 0)
			soa.grid_ends[prev_cell] = i;
		else
			soa.grid_ends[soa.grid_cells[Application::N - 1]] = Application::N;
	}
}

__global__ void calculateBoidsKernel(cudaArrays soa,
	Shoal::paramsStruct params,
	double d, glm::mat4* models,
	float x, float y, float z)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

#ifdef SNAP_TO_GRID
	int cell = calculate_grid_index(soa.positions[i]);
	int density = (int)glm::ceil(WORLD_WIDTH / params->visibility_radius);

	glm::vec3 pos = soa.positions[i];

	int gridX = (int)glm::floor((pos.x - LEFT_WALL + 1e-3) / params->visibility_radius);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL + 1e-3) / params->visibility_radius);
	int gridZ = (int)glm::floor((pos.z - BACK_WALL + 1e-3) / params->visibility_radius);

	soa.positions_bb[i] = soa.positions[i];
	soa.velocities_bb[i] = soa.velocities[i];

	/*
	soa.positions_bb[i] = glm::vec3(
		LEFT_WALL + MIN_GRID_R * gridX,
		DOWN_WALL + MIN_GRID_R * gridY,
		BACK_WALL + MIN_GRID_R * gridZ);
	*/
	//else
		//soa.positions_bb[i] = glm::vec3(x, y, z);//0.7f * glm::vec3(glm::sin(d), glm::cos(d));

	//apply_boid_rules(soa, params, i, 1);

#else

	glm::vec3 new_vel;
	glm::vec3 ret = apply_boid_rules(soa, params, i, 1);
	
	new_vel = soa.velocities[i] + ret;
	new_vel = turn_from_wall(soa.positions[i], new_vel, params);
	new_vel = speed_limit(new_vel, params);

	//printf("%f %f\n", new_vel.x, new_vel.y);

	glm::vec3 new_pos = soa.positions[i] + (float)d * new_vel;
	new_pos = teleport_through_wall(new_pos);

	if (glm::isnan(new_pos.x))
		new_pos.x = 0;
	if (glm::isnan(new_pos.y))
		new_pos.y = 0;
	if (glm::isnan(new_pos.z))
		new_pos.z = 0;
	if (glm::isnan(new_vel.x))
		new_vel.x = 0;
	if (glm::isnan(new_vel.y))
		new_vel.y = 0;
	if (glm::isnan(new_vel.z))
		new_vel.z = 0;

	soa.velocities_bb[i] = new_vel;
	soa.positions_bb[i] = new_pos;
	

#endif
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
