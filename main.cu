#include "framework.h"
#include <ctime>
#include <cstdlib>

void checkCudaError() 
{
	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "Error: %s\n", cudaGetErrorString(cudaStatus));
	}
}

void gpu(cpu_shoal*, double);

glm::mat3* model;
glm::vec2* positions;
glm::vec2* velocities;
glm::vec2* positions_bb;
glm::vec2* velocities_bb;
int* grid_cells;
int* grid_boids;
int* grid_starts;
int* grid_cellsizes;

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
	size_t int_size = N * sizeof(int);

	cudaMalloc(&model, mat_size);
	cudaMalloc(&positions, vec_size);
	cudaMalloc(&velocities, vec_size);
	cudaMalloc(&positions_bb, vec_size);
	cudaMalloc(&velocities_bb, vec_size);
	cudaMalloc(&grid_cells, int_size);
	cudaMalloc(&grid_boids, int_size);
	cudaMalloc(&grid_starts, int_size);
	cudaMalloc(&grid_cellsizes, int_size);
#endif

	double previousTime = glfwGetTime();
	while (!glfwWindowShouldClose(window))
	{
		double currentTime = glfwGetTime();
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

		processInput(window);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGui::SliderFloat("cohesion", &shoal->c, 0.0f, 0.5f);
		ImGui::SliderFloat("separation", &shoal->s, 0.0f, 0.5f);
		ImGui::SliderFloat("alignment", &shoal->a, 0.0f, 0.5f);
		ImGui::SliderFloat("max_speed", &shoal->max_speed, 0.5f, 1.0f);
		ImGui::SliderFloat("min_speed", &shoal->min_speed, 0.0f, 0.5f);
		ImGui::SliderFloat("visbility_radius", &shoal->visibility_radius, 0.0f, 0.5f);



		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);

#ifdef CPU
		shoal->update_boids(deltaTime);
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), &(shoal->model)[0], GL_DYNAMIC_DRAW);
#else
		gpu(shoal, deltaTime);
#endif

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
		showError();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();

		//for (;;);
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();

	delete shoal;
	cudaFree(model);
	cudaFree(positions);
	cudaFree(positions_bb);
	cudaFree(velocities);
	cudaFree(velocities_bb);
	cudaFree(grid_cells);
	cudaFree(grid_boids);
	cudaFree(grid_starts);
	cudaFree(grid_cellsizes);

	// tu powinno byc jakies zwalnianie cudy

	return 0;
}

struct compare_by_x {
	__host__ __device__
		bool operator()(const glm::vec2& a, const glm::vec2& b) const {
		return a.x < b.x;
	}
};

void gpu(cpu_shoal* shoal, double deltaTime)
{
	// INITIALIZE ---------------------------------------------------------
	size_t mat_size = N * sizeof(glm::mat3);
	size_t vec_size = N * sizeof(glm::vec2);
	size_t int_size = N * sizeof(int);

	glm::mat3* host_model = (glm::mat3*)malloc(mat_size);

	cudaMemcpy(positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(positions_bb, positions, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(velocities_bb, velocities, vec_size, cudaMemcpyDeviceToDevice);

	const int max_threads = 1024;
	int blocks_per_grid = (N + max_threads - 1) / max_threads;


	// GRID ----------------------------------------------------------------
	calculateGridKernel << <blocks_per_grid, max_threads >> > (
		grid_cells, grid_boids, positions);

	/*
	thrust::sort_by_key(thrust::device, grid_cells, grid_cells + N, grid_boids);

	int* grid_cpu = (int*)malloc(int_size);
	cudaMemcpy(grid_cpu, grid_cells, int_size, cudaMemcpyDeviceToHost);
	int* grid_boids_cpu = (int*)malloc(int_size);
	cudaMemcpy(grid_boids_cpu, grid_boids, int_size, cudaMemcpyDeviceToHost);

	for (int i = 0; i < N; ++i)
		std::cout << i << " " << grid_cpu[i] << ", " << grid_boids_cpu[i] << std::endl;

	//  do this on gpu
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t size = density * density;
	size_t num_bytes = size * sizeof(int);

	int* starts = (int*)malloc(num_bytes);
	int* sizes = (int*)malloc(num_bytes);
	std::memset(starts, -1, num_bytes);
	std::memset(sizes, -1, num_bytes);

	int start_index = 0;
	int current_cell = 0;
	int len = 1;
	int i = 0;

	while (i < size) 
	{
		int current_cell = grid_cpu[i];
		int start_index = i;
		int len = 1;  

		while (i + 1 < size && grid_cpu[i] == grid_cpu[i + 1]) {
			len++;
			i++;
		}

		sizes[current_cell] = len;
		starts[current_cell] = start_index;

		std::cout << i << " " << starts[i] << std::endl;
		i++;
	}
	
	cudaMemcpy(grid_starts, starts, num_bytes, cudaMemcpyHostToDevice);
	cudaMemcpy(grid_cellsizes, sizes, num_bytes, cudaMemcpyHostToDevice);

	free(grid_boids_cpu);
	free(grid_cpu);
	free(starts);
	free(sizes);

	*/

	// POSITIONS & VELOCITIES -----------------------------------------------
	calculateBoidsKernel << <blocks_per_grid, max_threads >> > (
		positions, velocities,
		positions_bb, velocities_bb,
		grid_cells, grid_boids,
		deltaTime);

	cudaMemcpy(positions, positions_bb, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(velocities, velocities_bb, vec_size, cudaMemcpyDeviceToDevice);

	// MODEL MATRICES -------------------------------------------------------
	calculateModelKernel << <blocks_per_grid, max_threads >> > (model, positions, velocities);

	cudaMemcpy(shoal->positions, positions_bb, vec_size, cudaMemcpyDeviceToHost);
	cudaMemcpy(shoal->velocities, velocities_bb, vec_size, cudaMemcpyDeviceToHost);

	cudaMemcpy(host_model, model, mat_size, cudaMemcpyDeviceToHost);
	glBufferData(GL_ARRAY_BUFFER, mat_size, host_model, GL_DYNAMIC_DRAW);

	free(host_model);
}
