#include "framework.h"
#pragma once

cudaError_t deployCuda(int* c, const int* a, const int* b, unsigned int size);
__global__ void kernel(int* c, const int* a, const int* b);
__global__ void calculateBoidsKernel(glm::vec2* pos, glm::vec2* vel,
	glm::vec2* pos_bb, glm::vec2* vel_bb, int*, int*, double);
__global__ void calculateModelKernel(glm::mat3* models, 
	glm::vec2* pos, glm::vec2* vel);
__global__ void calculateGridKernel(int* , int*, glm::vec2* pos);