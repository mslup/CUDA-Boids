#pragma once

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#define GLM_FORCE_CUDA

#include <thrust/sort.h>
#include <thrust/execution_policy.h>

#include <cuda_gl_interop.h>

#include "ImGUI/imgui.h"
#include "ImGUI/imgui_impl_glfw.h"
#include "ImGUI/imgui_impl_opengl3.h"

constexpr int WIDTH = 800;
constexpr int HEIGHT = 800;
constexpr float MIN_R = 0.02f;
constexpr float GRID_R = 2 * MIN_R;
constexpr float LEFT_WALL = -1;
constexpr float DOWN_WALL = -1;
constexpr float WORLD_WIDTH = 1 - LEFT_WALL;

//todo: move this to a different file
struct cudaArrays {
	glm::mat3* models;
	glm::vec2* positions;
	glm::vec2* velocities;
	glm::vec2* positions_bb;
	glm::vec2* velocities_bb;
	int* grid_cells;
	int* grid_boids;
	int* grid_starts; // inclusive
	int* grid_ends; // exclusive
};

#include "application.hpp"
#include "vao.hpp"
#include "window.hpp"
#include "shoal.hpp"
#include "shader.h"
#include "cuda.cuh"

void callKernels(int blocks_per_grid, int max_threads, double deltaTime, glm::mat3* models, Shoal *, cudaArrays);

//#define CPU 