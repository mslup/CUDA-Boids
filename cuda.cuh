#include "framework.h"
#pragma once

__global__ void calculateGridKernel(struct cudaArrays soa);
__global__ void calculateGridStartsKernel(struct cudaArrays soa);
__global__ void calculateBoidsKernel(cudaArrays soa, 
	Shoal::paramsStruct params, double, glm::mat4 *, float, float, float);
