#include "framework.h"
#include <stdio.h>

//#define NAIVE

// todo: add declarations in cuh; reclamp 
__device__ int calculate_grid_index(glm::vec2 pos)
{
	int gridX = (int)glm::floor((pos.x - LEFT_WALL) / GRID_R);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL) / GRID_R);

	//printf("(%f - %f) / %f = %f | %d\n", pos.x, LEFT_WALL, GRID_R, (pos.x - LEFT_WALL) / GRID_R, gridX);

	int gridSize = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	gridX = glm::clamp(gridX, 0, gridSize - 1);
	gridY = glm::clamp(gridY, 0, gridSize - 1);

	return gridY * gridSize + gridX;
}

struct boidParamsStruct {
	glm::vec2* separation_component;
	glm::vec2* velocity_sum;
	glm::vec2* position_sum;
	int* neighbors;
} boidParams;

// todo: everything to const ref
__device__ void iterate_through_cell(const cudaArrays& soa, int cell, int i, boidParamsStruct boidParams)
{
	glm::vec2* pos = soa.positions;
	glm::vec2* pos_bb = soa.positions_bb;
	glm::vec2* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* grid_ends = soa.grid_ends;
	int* boids = soa.grid_boids;

	// todo: merge R with visib_radius
	float radius_sq = R * R;

	int start = grid_starts[cell];

	if (start == -1) // empty cell
		return;

	//int len = grid_cellsizes[cell];
	int end = grid_ends[cell];
	for (int k = start; k < end; k++)
	{
		int j = boids[k];
		if (i == j) continue;

		glm::vec2 diff = pos[i] - pos[j];
		//float lensq = glm::dot(diff, diff);
		float len = glm::length(diff);
		glm::vec2 norm = glm::normalize(diff);

		if (len < R)//)lensq < radius_sq)
		{
      //soa.velocities_bb[boids[k]] = glm::vec2(1, 0);
			(*boidParams.separation_component) += norm / len;
			(*boidParams.velocity_sum) += vel[j];
			(*boidParams.position_sum) += pos[j];
			(*boidParams.neighbors)++;
		}
	}
}

__device__ glm::vec2 apply_boid_rules(cudaArrays soa, const cpu_shoal::paramsStruct& params, int i, double d)
{
	glm::vec2* pos = soa.positions;
	glm::vec2* pos_bb = soa.positions_bb;
	glm::vec2* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* boids = soa.grid_boids;

	glm::vec2 separation_component(0, 0);
	glm::vec2 velocity_sum(0, 0);
	glm::vec2 position_sum(0, 0);
	int neighbors = 0;
	float radius_sq = R * R;

	boidParamsStruct boidParams;

	boidParams.separation_component = &separation_component;
	boidParams.velocity_sum = &velocity_sum;
	boidParams.position_sum = &position_sum;
	boidParams.neighbors = &neighbors;

#ifdef NAIVE
	for (int j = 0; j < N; ++j)
	{
		float len = glm::length(pos[i] - pos[j]);
		if (i != j && len < R)
		{
			separation_component += pos[i] - pos[j];
			velocity_sum += vel[j];
			position_sum += pos[j];

			neighbors++;
		}
	}
#else
	int density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	int cell = calculate_grid_index(pos[i]);
	int gridX = cell % density;
	int gridY = cell / density;

	// todo: calculate this beforehand? sort a duplicate buffer?
	int start_offset, horizontal_offset, vertical_offset;

	if (pos[i].x >= LEFT_WALL + (gridX + 0.5) * GRID_R)
	{
		start_offset = 0;
		horizontal_offset = 1;
	}
	else
	{
		start_offset = -1;
		horizontal_offset = -1;
	}

	if (pos[i].y >= DOWN_WALL + (gridY + 0.5) * GRID_R)
		vertical_offset = density;
	else
		vertical_offset = -density;

	iterate_through_cell(soa, cell, i, boidParams);

	if ((cell + horizontal_offset) / density == cell / density)
		iterate_through_cell(soa, cell + horizontal_offset, i, boidParams);

	if (cell + vertical_offset >= 0 && cell + vertical_offset < density * density)
		iterate_through_cell(soa, cell + vertical_offset, i, boidParams);

	if (cell + vertical_offset + horizontal_offset >= 0
		&& cell + vertical_offset + horizontal_offset < density * density
		&& (cell + vertical_offset + horizontal_offset) / density == (cell + vertical_offset) / density)
		iterate_through_cell(soa, cell + vertical_offset + horizontal_offset, i, boidParams);
#endif

	glm::vec2 alignment_component;
	glm::vec2 cohesion_component;

	if (neighbors == 0)
		return glm::vec2(0, 0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - vel[i];
	cohesion_component = position_sum - pos[i];

	return (float)d *
		(params.s * separation_component
			+ params.a * alignment_component
			+ params.c * cohesion_component);
}

__device__ glm::vec2 speed_limit(glm::vec2 vel, const cpu_shoal::paramsStruct& params)
{
	if (glm::length(vel) < params.min_speed)
		return params.min_speed * glm::normalize(vel);
	if (glm::length(vel) > params.max_speed)
		return params.max_speed * glm::normalize(vel);

	return vel;
}

__device__ glm::vec2 turn_from_wall(glm::vec2 pos, glm::vec2 vel, const cpu_shoal::paramsStruct& params)
{
	float dx_right = 1 - pos.x;
	float dx_left = pos.x + 1;
	float dy_up = 1 - pos.y;
	float dy_down = pos.y + 1;

	float len = glm::length(vel);

	glm::vec2 vel_change = glm::vec2(0, 0);

	if (dx_right < params.margin)
		vel_change.x -= params.turn * len / (dx_right * dx_right);
	if (dx_left < params.margin)
		vel_change.x += params.turn * len / (dx_left * dx_left);
	if (dy_up < params.margin)
		vel_change.y -= params.turn * len / (dy_up * dy_up);
	if (dy_down < params.margin)
		vel_change.y += params.turn * len / (dy_down * dy_down);

	return vel + vel_change;
}

__device__ glm::vec2 teleport_through_wall(glm::vec2 pos)
{
	glm::vec2 ret = pos;

	if (pos.x > 1)
		ret.x = -1;
	if (pos.y > 1)
		ret.y = -1;
	if (pos.x < -1)
		ret.x = 1;
	if (pos.y < -1)
		ret.y = 1;

	return ret;
}

__global__ void calculateGridKernel(cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

	soa.grid_cells[i] = calculate_grid_index(soa.positions[i]);
	soa.grid_boids[i] = i;

	// these are getting sorted later
}

__global__ void calculateGridStartsKernel(struct cudaArrays soa)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	int cell = soa.grid_cells[i];
	int prev_cell = i != 0 ? soa.grid_cells[i - 1] : -1;
	if (i == 0 || cell != prev_cell)
	{
		// todo: there's a bottleneck
		soa.grid_starts[cell] = i;
		if (i != 0)
			soa.grid_ends[prev_cell] = i;
	}
}

__global__ void calculateBoidsKernel(cudaArrays soa, cpu_shoal::paramsStruct params, double d, float x, float y)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

#ifdef SNAP_TO_GRID
	glm::vec2* pos = soa.positions;
	glm::vec2* vel = soa.velocities;
	int* grid_starts = soa.grid_starts;
	int* grid_cells = soa.grid_cells;
	int* grid_cellsizes = soa.grid_cellsizes;
	int* boids = soa.grid_boids;

	int cell = calculate_grid_index(pos[i]);
	int density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	int gridX = cell % density;
	int gridY = cell / density;

	gridX = (int)glm::floor((pos[i].x - LEFT_WALL + 1e-3) / GRID_R);
	gridY = (int)glm::floor((pos[i].y - DOWN_WALL + 1e-3) / GRID_R);

	if (i != 42)
	{
		//soa.positions_bb[i] = glm::vec2(LEFT_WALL + GRID_R * gridX, DOWN_WALL + GRID_R * gridY);
	}
	else
		soa.positions_bb[i] = glm::vec2(x, y);//0.7f * glm::vec2(glm::sin(d), glm::cos(d));

	apply_boid_rules(soa, params, i, 1);
#else
	glm::vec2 new_vel;

	new_vel = soa.velocities[i]
		+ apply_boid_rules(soa, params, i, 1);
	new_vel = speed_limit(new_vel, params);
	new_vel = turn_from_wall(soa.positions[i], new_vel, params);

	soa.velocities_bb[i] = new_vel;
	glm::vec2 new_pos = soa.positions[i] + (float)d * new_vel;
	new_pos = teleport_through_wall(new_pos);

	soa.positions_bb[i] = new_pos;

	if (new_pos.x < -1 || new_pos.x > 1 || new_pos.y < -1 || new_pos.y > 1)
		printf("%f, %f\n", new_pos.x, new_pos.y);
#endif
	__syncthreads();

	soa.positions[i] = soa.positions_bb[i];
	soa.velocities[i] = soa.velocities_bb[i];

	glm::vec2 v = glm::normalize(soa.velocities[i]);
	glm::vec2 vT = glm::vec2(v.y, -v.x);
	soa.models[i] = glm::mat3(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(soa.positions[i], 1.0f));
}
