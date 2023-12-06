#include "framework.h"
#pragma once

cudaError_t deployCuda(int* c, const int* a, const int* b, unsigned int size);
__global__ void kernel(int* c, const int* a, const int* b);
__global__ void kernel_tmp(glm::mat3 *models, cpu_shoal *shoal);