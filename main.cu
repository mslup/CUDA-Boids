#include "framework.h"
#include <ctime>
#include <cstdlib>

void gpu(cpu_shoal *, double);

glm::mat3* model;
glm::vec2* positions;
glm::vec2* velocities;
glm::vec2* positions_bb;
glm::vec2* velocities_bb;
int* grid;

int main()
{

#ifndef CPU
	cudaSetDevice(0);
#endif

	//srand(time(NULL));

	initWindow();
	cpu_shoal* shoal = new cpu_shoal();

	create_buffer_objects(shoal);

	Shader shader("./vertex.glsl", "./fragment.glsl");
	shader.use();

#ifdef CPU
	shader.setFloat3("boidColor", 0.2f, 0.7f, 0.4f);
#else
	shader.setFloat3("boidColor", 0.9f, 0.5f, 0.0f);

	size_t mat_size = N * sizeof(glm::mat3);
	size_t vec_size = N * sizeof(glm::vec2);

	cudaMalloc(&model, mat_size);
	cudaMalloc(&positions, vec_size);
	cudaMalloc(&velocities, vec_size);
	cudaMalloc(&positions_bb, vec_size);
	cudaMalloc(&velocities_bb, vec_size);
	cudaMalloc(&grid, N * sizeof(int));

#endif

	double previousTime = glfwGetTime();
	while (!glfwWindowShouldClose(window))
	{
		double currentTime = glfwGetTime(); 
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

		processInput(window);

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);

#ifdef CPU
		shoal->update_boids();
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), &(shoal->model)[0], GL_DYNAMIC_DRAW);
#else
		gpu(shoal, deltaTime);
#endif

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
		showError();

		glfwSwapBuffers(window);
		glfwPollEvents();

		//for (;;);
	}

	glfwTerminate();
	
	delete shoal;
	cudaFree(model);
	cudaFree(positions);
	cudaFree(positions_bb);
	cudaFree(velocities);
	cudaFree(velocities_bb);

	// tu powinno byc jakies zwalnianie cudy

	return 0;
}

void gpu(cpu_shoal *shoal, double deltaTime)
{
	size_t mat_size = N * sizeof(glm::mat3);
	size_t vec_size = N * sizeof(glm::vec2);

	glm::mat3* host_model = (glm::mat3*)malloc(mat_size);

	cudaMemcpy(positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(positions_bb, positions, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(velocities_bb, velocities, vec_size, cudaMemcpyDeviceToDevice);

	const int max_threads = 1024;
	int blocks_per_grid = (N + max_threads - 1) / max_threads;

	calculateBoidsKernel << <blocks_per_grid, max_threads >> > (
		positions, velocities,
		positions_bb, velocities_bb, 
		grid,
		deltaTime);

	cudaMemcpy(positions, positions_bb, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(velocities, velocities_bb, vec_size, cudaMemcpyDeviceToDevice);

	calculateModelKernel << <blocks_per_grid, max_threads >> > (model, positions, velocities);

	cudaMemcpy(shoal->positions, positions_bb, vec_size, cudaMemcpyDeviceToHost);
	cudaMemcpy(shoal->velocities, velocities_bb, vec_size, cudaMemcpyDeviceToHost);

	cudaMemcpy(host_model, model, mat_size, cudaMemcpyDeviceToHost);
	glBufferData(GL_ARRAY_BUFFER, mat_size, host_model, GL_DYNAMIC_DRAW);

	free(host_model);
	
}
