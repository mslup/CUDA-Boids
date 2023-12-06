#include "framework.h"

#pragma once

extern GLFWwindow* window;
extern unsigned int VBO, VAO;
extern unsigned int modelVBO;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

void processInput(GLFWwindow* window);

void showError();

GLFWwindow* initWindow();

void create_buffer_objects(cpu_shoal* shoal);

void render(cpu_shoal *shoal);

