#include "framework.h"
#include <stdio.h>

__device__ glm::vec2 apply_boid_rules(glm::vec2* pos, glm::vec2* vel, int i, double d)
{
	float visibility_radius = 1e-1;
	float s = 0.01;//3e-2;
	float a = 0.1;//8e-2;
	float c = 0.005;//9e-2;

	glm::vec2 separation_component(0, 0);
	glm::vec2 velocity_sum(0, 0);
	glm::vec2 position_sum(0, 0);
	int neighbors = 0;

	for (int j = 0; j < N; ++j)
	{
		float len = glm::length(pos[i] - pos[j]);
		if (i != j && len < visibility_radius)
		{
			separation_component += pos[i] - pos[j];
			velocity_sum += vel[j];
			position_sum += pos[j];

			neighbors++;
		}
	}

	glm::vec2 alignment_component;
	glm::vec2 cohesion_component;

	if (neighbors == 0)
		return glm::vec2(0, 0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - vel[i];
	cohesion_component = position_sum - pos[i];

	return (float)d* (s * separation_component + a * alignment_component + c * cohesion_component);
}

__device__ glm::vec2 speed_limit(glm::vec2 vel)
{
	float max_speed = 0.9;
	float min_speed = 0.1;

	if (glm::length(vel) < min_speed)
		return min_speed * glm::normalize(vel);
	if (glm::length(vel) > max_speed)
		return max_speed * glm::normalize(vel);

	return vel;
}

__device__ glm::vec2 turn_from_wall(glm::vec2 pos, glm::vec2 vel)
{
	float margin = 0.2f;
	float turn = 5e-4;

	float dx_right = 1 - pos.x;
	float dx_left = pos.x + 1;
	float dy_up = 1 - pos.y;
	float dy_down = pos.y + 1;

	float len = glm::length(vel);

	glm::vec2 vel_change = glm::vec2(0, 0);

	if (dx_right < margin)
		vel_change.x -= turn * len / (dx_right * dx_right);
	if (dx_left < margin)
		vel_change.x += turn * len / (dx_left * dx_left);
	if (dy_up < margin)
		vel_change.y -= turn * len / (dy_up * dy_up);
	if (dy_down < margin)
		vel_change.y += turn * len / (dy_down * dy_down);

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

__device__ int calculate_grid_index(glm::vec2 pos)
{
	int gridX = (int)glm::floor((pos.x - LEFT_WALL) / GRID_R);
	int gridY = (int)glm::floor((pos.y - DOWN_WALL) / GRID_R);

	int gridSize = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	gridX = glm::clamp(gridX, 0, gridSize - 1);
	gridY = glm::clamp(gridY, 0, gridSize - 1);

	return gridY * gridSize + gridX;
}


__global__ void calculateGridKernel(int* grid_cells, int* grid_boids, glm::vec2* pos)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

	grid_cells[i] = calculate_grid_index(pos[i]);
	grid_boids[i] = i;
}

__global__ void calculateBoidsKernel(
	glm::vec2* pos, glm::vec2* vel, 
	glm::vec2* pos_bb, glm::vec2* vel_bb, 
	int* grid_cells, int* grid_boids, double d)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

	glm::vec2 new_vel;

	new_vel = vel[i] + apply_boid_rules(pos, vel, i, 1);
	new_vel = speed_limit(new_vel);
	new_vel = turn_from_wall(pos[i], new_vel);

	vel_bb[i] = new_vel;
	glm::vec2 new_pos = pos[i] + (float)d * new_vel;
	new_pos = teleport_through_wall(new_pos);
		
	pos_bb[i] = new_pos;

	
	//int gridX = grid_cells[i] % (int)glm::ceil(WORLD_WIDTH / GRID_R);
	//int gridY = grid_cells[i] / (int)glm::ceil(WORLD_WIDTH / GRID_R);
	//pos_bb[i] = glm::vec2(LEFT_WALL + GRID_R * gridX, 
	//	DOWN_WALL + GRID_R * gridY);
	

	//syncthreads instead of two kernels
}

__global__ void calculateModelKernel(glm::mat3* models, glm::vec2* pos, glm::vec2* vel)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;

	if (i >= N)
		return;

	// calculate model matrix
	glm::vec2 v = glm::normalize(vel[i]);
	glm::vec2 vT = glm::vec2(v.y, -v.x);
	models[i] = glm::mat3(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(pos[i], 1.0f));
}
