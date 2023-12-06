#pragma once

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include <glm/glm.hpp>
#define GLM_FORCE_CUDA

#include <thrust/sort.h>
#include <thrust/execution_policy.h>

//#include <cuda_gl_interop.h>

constexpr int N = 100;
constexpr int WIDTH = 800;
constexpr int HEIGHT = 800;
constexpr float R = 0.2f;
constexpr float GRID_R = 2 * R;
constexpr float LEFT_WALL = -1;
constexpr float DOWN_WALL = -1;
constexpr float WORLD_WIDTH = 1 - LEFT_WALL;

#include "shader.h"
#include "cpu_shoal.hpp"
#include "display.hpp"
#include "cuda.cuh"

//#define CPU 