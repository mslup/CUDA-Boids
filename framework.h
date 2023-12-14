#pragma once

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

constexpr int WIDTH = 800;
constexpr int HEIGHT = 800;
constexpr float LEFT_WALL = -2;
constexpr float DOWN_WALL = -2;
constexpr float BACK_WALL = -2;
constexpr float WORLD_WIDTH = -2 * LEFT_WALL;

//todo: move this to a different file
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

void callKernels(int blocks_per_grid, int max_threads,
	double deltaTime, glm::mat4* models, Shoal*, cudaArrays
	, float x, float y, float z);

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

//#define CPU 