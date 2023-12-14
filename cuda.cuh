#include "framework.h"
#pragma once

struct boidParamsStruct {
	glm::vec3* separation_component;
	glm::vec3* velocity_sum;
	glm::vec3* position_sum;
	int* neighbors;
};

__device__ int calculate_grid_index(glm::vec3 pos, float grid_radius);
__device__ void iterate_through_cell(const cudaArrays& soa, 
	const Shoal::behaviourParamsStruct& behaviourParams,
	int cell, int i, boidParamsStruct boidParams);
__device__ glm::vec3 apply_boid_rules(const cudaArrays& soa, 
	const Shoal::behaviourParamsStruct& behaviourParams,
	int i, double d);
__device__ glm::vec3 speed_limit(glm::vec3 vel, 
	const Shoal::behaviourParamsStruct& behaviourParams);
__device__ glm::vec3 turn_from_wall(glm::vec3 pos, glm::vec3 vel, 
	const Shoal::behaviourParamsStruct& behaviourParams, 
	float d);

__global__ void calculateGridKernel(const cudaArrays soa, float radius);
__global__ void calculateGridStartsKernel(const cudaArrays soa);
__global__ void calculateBoidsKernel(const cudaArrays soa, 
	Shoal::behaviourParamsStruct behaviourParams, double, glm::mat4 *);
