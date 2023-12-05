#pragma once

//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"

#include <stdio.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include <glm/glm.hpp>
//#define GLM_FORCE_CUDA

//#include <cuda_gl_interop.h>

constexpr int N = 100;
constexpr int WIDTH = 800;
constexpr int HEIGHT = 800;

#include "shader.h"
#include "cpu_shoal.hpp"
#include "display.hpp"
#include "cuda.h"