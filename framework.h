#pragma once

// #########################
//#define CPU 
// Build > Rebuild solution
// #########################

#ifdef CPU
#define BOID_COUNT 4000
#else
#define BOID_COUNT 50000
//#define NAIVE
#endif

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#define GLM_FORCE_CUDA

#include <thrust/sort.h>
#include <thrust/execution_policy.h>

#include <cuda_gl_interop.h>

#include "ImGUI/imgui.h"
#include "ImGUI/imgui_impl_glfw.h"
#include "ImGUI/imgui_impl_opengl3.h"

struct cudaArrays {
	glm::mat4* models;
	glm::vec3* positions;
	glm::vec3* velocities;
	glm::vec3* positions_bb;
	glm::vec3* velocities_bb;
	int* grid_cells;
	int* grid_boids;
	int* grid_starts; // inclusive
	int* grid_ends; // exclusive
};

#include "application.hpp"
#include "vao.hpp"
#include "window.hpp"
#include "shoal.hpp"
#include "shader.hpp"
#include "camera.hpp"
#include "cuda.cuh"

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

