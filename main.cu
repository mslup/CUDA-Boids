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

//todo: not global
struct cudaArrays cudaArrays;

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
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	cudaMalloc(&cudaArrays.models, mat_size);
	cudaMalloc(&cudaArrays.positions, vec_size);
	cudaMalloc(&cudaArrays.velocities, vec_size);
	cudaMalloc(&cudaArrays.positions_bb, vec_size);
	cudaMalloc(&cudaArrays.velocities_bb, vec_size);
	cudaMalloc(&cudaArrays.grid_cells, int_size);
	cudaMalloc(&cudaArrays.grid_boids, int_size);
	cudaMalloc(&cudaArrays.grid_starts, grid_size);
	cudaMalloc(&cudaArrays.grid_cellsizes, grid_size);
	cudaMalloc(&cudaArrays.grid_ends, grid_size);
#endif

	glfwSwapInterval(0);

	int num_frames = 0;
	int fps = 0;
	double previousTime = glfwGetTime();
	double previousFpsTime = previousTime;
	while (!glfwWindowShouldClose(window))
	{
		double currentTime = glfwGetTime();
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

		processInput(window);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGui::SliderFloat("cohesion", &shoal->params.c, 0.0f, 0.5f);
		ImGui::SliderFloat("separation", &shoal->params.s, 0.0f, 0.5f);
		ImGui::SliderFloat("alignment", &shoal->params.a, 0.0f, 0.5f);
		ImGui::SliderFloat("max_speed", &shoal->params.max_speed, 0.5f, 1.0f);
		ImGui::SliderFloat("min_speed", &shoal->params.min_speed, 0.0f, 0.5f);
		ImGui::SliderFloat("visbility_radius", &shoal->params.visibility_radius, 0.0f, 0.5f);

		num_frames++;
		if (currentTime - previousFpsTime >= 1.0)
		{
			// printf and reset timer
			//printf("%d fps\n", num_frames);
			fps = num_frames;
			num_frames = 0;
			previousFpsTime += 1.0;
		}
		ImGui::Text("%d fps", fps);

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);

#ifdef CPU
		shoal->update_boids(deltaTime);
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->models), &(shoal->models)[0], GL_DYNAMIC_DRAW);
#else
		gpu(shoal, deltaTime);
#endif

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
		showError();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();

		//while (true);
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();

	delete shoal;
	cudaFree(cudaArrays.models);
	cudaFree(cudaArrays.positions);
	cudaFree(cudaArrays.positions_bb);
	cudaFree(cudaArrays.velocities);
	cudaFree(cudaArrays.velocities_bb);
	cudaFree(cudaArrays.grid_cells);
	cudaFree(cudaArrays.grid_boids);
	cudaFree(cudaArrays.grid_starts);
	cudaFree(cudaArrays.grid_cellsizes);
	cudaFree(cudaArrays.grid_ends);

	//TODO: tu powinno byc jakies zwalnianie cudy

	return 0;
}


void gpu(cpu_shoal* shoal, double deltaTime)
{


	static float x = 0, static float y = 0;
	ImGui::SliderFloat("x", &x, -1.0f, 1.0f);
	ImGui::SliderFloat("y", &y, -1.0f, 1.0f);

	// INITIALIZE ---------------------------------------------------------
	size_t mat_size = N * sizeof(glm::mat3);
	size_t vec_size = N * sizeof(glm::vec2);
	size_t int_size = N * sizeof(int);
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	glm::mat3* host_model = (glm::mat3*)malloc(mat_size);

	cudaMemcpy(cudaArrays.positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(cudaArrays.velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(cudaArrays.positions_bb, cudaArrays.positions, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(cudaArrays.velocities_bb, cudaArrays.velocities, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemset(cudaArrays.grid_boids, 0, int_size);
	cudaMemset(cudaArrays.grid_cells, 0, int_size);
	cudaMemset(cudaArrays.grid_starts, -1, grid_size);
	cudaMemset(cudaArrays.grid_cellsizes, 0, grid_size);	
	cudaMemset(cudaArrays.grid_ends, -1, grid_size);	

	const int max_threads = 1024;
	int blocks_per_grid = (N + max_threads - 1) / max_threads;

	// GRID ----------------------------------------------------------------
	calculateGridKernel << <blocks_per_grid, max_threads >> > (cudaArrays);

	thrust::sort_by_key(thrust::device, cudaArrays.grid_cells, cudaArrays.grid_cells + N, cudaArrays.grid_boids);

	calculateGridStartsKernel << <blocks_per_grid, max_threads >> > (cudaArrays);

	/*
	int* grid_cpu = (int*)malloc(int_size);
	cudaMemcpy(grid_cpu, cudaArrays.grid_cells, int_size, cudaMemcpyDeviceToHost);
	int* grid_boids_cpu = (int*)malloc(int_size);
	cudaMemcpy(grid_boids_cpu, cudaArrays.grid_boids, int_size, cudaMemcpyDeviceToHost);

	//TODO: do this on gpu
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t size = density * density;
	size_t num_bytes = size * sizeof(int);

	int* starts = new int[size];
	int* sizes = new int[size];
	std::memset(starts, -1, num_bytes);
	std::memset(sizes, 0, num_bytes);

	int start_index = 0;
	int current_cell = 0;
	int len = 1;
	int i = 0;
	
	while (i < N)
	{
		int current_cell = grid_cpu[i];
		int start_index = i;
		int len = 1;

		while (i + 1 < N && grid_cpu[i] == grid_cpu[i + 1]) {
			len++;
			i++;
		}

		sizes[current_cell] = len;
		starts[current_cell] = start_index;

		//std::cout << current_cell << ": " << starts[current_cell] << " " << sizes[current_cell] << std::endl;
		i++;
	}

	cudaMemcpy(cudaArrays.grid_starts, starts, num_bytes, cudaMemcpyHostToDevice);
	cudaMemcpy(cudaArrays.grid_cellsizes, sizes, num_bytes, cudaMemcpyHostToDevice);
	*/

	// POSITIONS & VELOCITIES -----------------------------------------------
	calculateBoidsKernel << <blocks_per_grid, max_threads >> > (cudaArrays, shoal->params, deltaTime, x, y);

	cudaMemcpy(cudaArrays.positions, cudaArrays.positions_bb, vec_size, cudaMemcpyDeviceToDevice);
	cudaMemcpy(cudaArrays.velocities, cudaArrays.velocities_bb, vec_size, cudaMemcpyDeviceToDevice);

	// MODEL MATRICES -------------------------------------------------------
	calculateModelKernel << <blocks_per_grid, max_threads >> > (cudaArrays);

	cudaMemcpy(shoal->positions, cudaArrays.positions_bb, vec_size, cudaMemcpyDeviceToHost);
	cudaMemcpy(shoal->velocities, cudaArrays.velocities_bb, vec_size, cudaMemcpyDeviceToHost);

	cudaMemcpy(host_model, cudaArrays.models, mat_size, cudaMemcpyDeviceToHost);
	glBufferData(GL_ARRAY_BUFFER, mat_size, host_model, GL_DYNAMIC_DRAW);

	free(host_model);
	//free(grid_boids_cpu);
	//free(grid_cpu);
	//delete[] starts;
	//delete[] sizes;
}
