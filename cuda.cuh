#include "framework.h"
#pragma once

__global__ void calculateGridKernel(struct cudaArrays soa);
__global__ void calculateBoidsKernel(struct cudaArrays soa, 
	cpu_shoal::paramsStruct params, double, float, float);
__global__ void calculateModelKernel(struct cudaArrays soa);