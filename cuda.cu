#include "framework.h"
#include <stdio.h>

//#define NAIVE
#define SNAP_TO_GRID

// todo: add declarations in cuh; reclamp 
__device__ int calculate_grid_index(glm::vec3 pos)
{
	int gridX = (int)glm::floor((pos.x - LEFT_WALL) / GRID_R);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL) / GRID_R);
	int gridZ = (int)glm::floor((pos.z - BACK_WALL) / GRID_R);

	//printf("(%f - %f) / %f = %f | %d\n", pos.x, LEFT_WALL, GRID_R, (pos.x - LEFT_WALL) / GRID_R, gridX);

	int gridSize = (int)glm::ceil(WORLD_WIDTH / GRID_R);
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
__device__ void iterate_through_cell(const cudaArrays& soa, int cell, int i, boidParamsStruct boidParams)
{
	glm::vec3* pos = soa.positions;
	glm::vec3* pos_bb = soa.positions_bb;
	glm::vec3* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* grid_ends = soa.grid_ends;
	int* boids = soa.grid_boids;

	// todo: merge R with visib_radius
	float radius_sq = MIN_R * MIN_R;

	int start = grid_starts[cell];

	if (start == -1) // empty cell
		return;

	//int len = grid_cellsizes[cell];
	int end = grid_ends[cell];
	for (int k = start; k < end; k++)
	{
		int j = boids[k];
		if (i == j) continue;

		glm::vec3 diff = pos[i] - pos[j];
		float len = glm::length(diff);
		glm::vec3 norm = glm::normalize(diff);

		if (len < MIN_R)
		{
			soa.velocities_bb[boids[k]] = glm::vec3(0, 0, 1);

			/*
			(*boidParams.separation_component) += norm / len;
			(*boidParams.velocity_sum) += vel[j];
			(*boidParams.position_sum) += pos[j];
			(*boidParams.neighbors)++;
			*/
		}
	}
}

__device__ glm::vec3 apply_boid_rules(cudaArrays soa, const Shoal::paramsStruct& params, int i, double d)
{
	glm::vec3* pos = soa.positions;
	glm::vec3* pos_bb = soa.positions_bb;
	glm::vec3* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* boids = soa.grid_boids;

	glm::vec3 separation_component(0);
	glm::vec3 velocity_sum(0);
	glm::vec3 position_sum(0);
	int neighbors = 0;
	float radius_sq = MIN_R * MIN_R;

	boidParamsStruct boidParams;

	boidParams.separation_component = &separation_component;
	boidParams.velocity_sum = &velocity_sum;
	boidParams.position_sum = &position_sum;
	boidParams.neighbors = &neighbors;

#ifdef NAIVE
	for (int j = 0; j < Application::N; ++j)
	{
		glm::vec3 diff = pos[i] - pos[j];
		float len = glm::length(diff);
		glm::vec3 norm = glm::normalize(diff);

		if (i != j && len < MIN_R)
		{
			separation_component += norm / len;
			velocity_sum += vel[j];
			position_sum += pos[j];

			neighbors++;
		}
	}
#else
	int density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	int cell = calculate_grid_index(pos[i]);
	int gridX = cell % density;
	int gridY = (cell / density) % density;
	int gridZ = cell / (density * density);

	// todo: calculate this beforehand? sort a duplicate buffer?
	int start_offset, x_offset, y_offset, z_offset;

	if (pos[i].x >= LEFT_WALL + (gridX + 0.5) * GRID_R)
	{
		start_offset = 0;
		x_offset = 1;
	}
	else
	{
		start_offset = -1;
		x_offset = -1;
	}

	if (pos[i].y >= DOWN_WALL + (gridY + 0.5) * GRID_R)
		y_offset = density;
	else
		y_offset = -density;

	iterate_through_cell(soa, cell, i, boidParams);

	if ((cell + x_offset) / density == cell / density)
		iterate_through_cell(soa, cell + x_offset, i, boidParams);

	if (cell + y_offset >= 0 && cell + y_offset < density * density)
		iterate_through_cell(soa, cell + y_offset, i, boidParams);

	if (cell + y_offset + x_offset >= 0
		&& cell + y_offset + x_offset < density * density
		&& (cell + y_offset + x_offset) / density == (cell + y_offset) / density)
		iterate_through_cell(soa, cell + y_offset + x_offset, i, boidParams);
#endif

	glm::vec3 alignment_component;
	glm::vec3 cohesion_component;

	if (neighbors == 0)
		return glm::vec3(0, 0, 0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - vel[i];
	cohesion_component = position_sum - pos[i];

	return (float)d *
		(params.s * separation_component
			+ params.a * alignment_component
			+ params.c * cohesion_component);
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

__global__ void calculateGridKernel(struct cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	//printf("calculate grid\n");

	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * density * sizeof(int);

	//printf("grid size %d\n", grid_size);
	//printf("zjeb %d\n", i);

	//int d = soa.grid_cells[i];

	soa.grid_cells[i] = calculate_grid_index(soa.positions[i]);
	soa.grid_boids[i] = i;
	
	// these are getting sorted later
}

__global__ void calculateGridStartsKernel(struct cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	//printf("calculate grid starts\n");

	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * density * sizeof(int);

	//printf("grid_size: %d\n", grid_size);

	int cell = soa.grid_cells[i];
	int prev_cell = i != 0 ? soa.grid_cells[i - 1] : -1;

	//printf("cell: %d\n", cell);

	/*
	if (i == 0 || cell != prev_cell)
	{
		// todo: there's a bottleneck
		soa.grid_starts[cell] = i;
		if (i != 0)
			soa.grid_ends[prev_cell] = i;
	}
	*/
	//printf("pa pa: %d\n", cell);
}

__global__ void calculateBoidsKernel(struct cudaArrays soa,  Shoal::paramsStruct params, double d, glm::mat4* models)
{
	printf("calcualte boids???\n");
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= Application::N)
		return;

	//printf("calcualte boids\n");
	//printf("%f, %f, %f\n", soa.positions[i].x, soa.positions[i].y, soa.positions[i].z);
	//printf("%f, %f, %f\n", soa.positions_bb[i].x, soa.positions_bb[i].y, soa.positions_bb[i].z);

#ifdef SNAP_TO_GRID
	glm::vec3* pos = soa.positions;
	glm::vec3* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* boids = soa.grid_boids;

	int cell = calculate_grid_index(pos[i]);
	int density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	
	//int jebanegowno = (int)glm::floor((pos[i].z - LEFT_WALL + 1e-3) / GRID_R);

	/*
	int gridY = (int)glm::floor((pos[i].y - DOWN_WALL + 1e-3) / GRID_R);
	*/
	int gridX = (int)glm::floor((pos[i].x - LEFT_WALL + 1e-3) / GRID_R);
	int gridZ = (int)glm::floor((pos[i].z - BACK_WALL + 1e-3) / GRID_R);
	int jebanegowno2 = (int)glm::floor((pos[i].y - DOWN_WALL + 1e-3) / GRID_R);

	//soa.positions_bb[i] = soa.positions[i];
	//soa.velocities_bb[i] = soa.velocities[i];

	//soa.positions_bb[i] = glm::vec3(LEFT_WALL + GRID_R * gridX, DOWN_WALL + GRID_R * gridY, 0);//BACK_WALL + GRID_R * gridZ);
	//else
		//soa.positions_bb[i] = glm::vec3(x, y, z);//0.7f * glm::vec3(glm::sin(d), glm::cos(d));

	//apply_boid_rules(soa, params, i, 1);
	
#else
	glm::vec3 new_vel;

	new_vel = soa.velocities[i]
		+ apply_boid_rules(soa, params, i, 1);
	new_vel = speed_limit(new_vel, params);
	new_vel = turn_from_wall(soa.positions[i], new_vel, params);

	soa.velocities_bb[i] = new_vel;
	glm::vec3 new_pos = soa.positions[i] + (float)d * new_vel;
	new_pos = teleport_through_wall(new_pos);

	soa.positions_bb[i] = new_pos;

	if (new_pos.x < -1 || new_pos.x > 1 || new_pos.y < -1 || new_pos.y > 1)
		printf("%f, %f\n", new_pos.x, new_pos.y);
#endif
	__syncthreads();

	//soa.positions[i] = soa.positions_bb[i];
	//soa.velocities[i] = soa.velocities_bb[i];

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
	
	/*
	models[i] = glm::mat4(
		glm::vec4(1, 0, 0, 0),
		glm::vec4(0, 1, 0, 0),
		glm::vec4(0, 0, 1, 0),
		glm::vec4(soa.positions[i], 1)
	);
	*/
}
