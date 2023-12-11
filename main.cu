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

bool pause = false;

glm::vec3 backColor = glm::vec3(84, 65, 78);
glm::vec3 boidColor = glm::vec3(167, 144, 165);

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
	shader.setFloat3("boidColor", 
		boidColor.r / 255.0f, 
		boidColor.g / 255.0f, 
		boidColor.b / 255.0f);

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
	cudaMalloc(&cudaArrays.grid_ends, grid_size);

	cudaMemcpy(cudaArrays.positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(cudaArrays.velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);

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

		if (ImGui::Button("Pause"))
		{
			pause = !pause;
		}

		ImGui::SliderFloat("cohesion", &shoal->params.c, 0.0f, 0.5f);
		ImGui::SliderFloat("separation", &shoal->params.s, 0.0f, 0.1f);
		ImGui::SliderFloat("alignment", &shoal->params.a, 0.0f, 0.5f);
		ImGui::SliderFloat("max_speed", &shoal->params.max_speed, 0.5f, 1.0f);
		ImGui::SliderFloat("min_speed", &shoal->params.min_speed, 0.0f, 0.5f);
		ImGui::SliderFloat("visbility_radius", &shoal->params.visibility_radius, 0.0f, 0.5f);

		num_frames++;
		if (currentTime - previousFpsTime >= 1.0)
		{
			fps = num_frames;
			num_frames = 0;
			previousFpsTime += 1.0;
		}
		ImGui::Text("%d fps", fps);

		glClearColor(backColor.r / 255.0f, 
			backColor.g / 255.0f, 
			backColor.b / 255.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
		glBufferData(GL_ARRAY_BUFFER, mat_size, 0, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0); // ???????????????
		cudaGraphicsGLRegisterBuffer(&cudaVBO, modelVBO, cudaGraphicsMapFlagsWriteDiscard);

		if (!pause)
		{
#ifdef CPU
		shoal->update_boids(deltaTime);
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->models), &(shoal->models)[0], GL_DYNAMIC_DRAW);
#else
		gpu(shoal, deltaTime);
#endif
		}

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
		showError();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
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
	cudaFree(cudaArrays.grid_ends);

	//TODO: tu powinno byc jakies zwalnianie cudy

	return 0;
}


void gpu(cpu_shoal* shoal, double deltaTime)
{
	size_t mat_size = N * sizeof(glm::mat3);
	size_t vec_size = N * sizeof(glm::vec2);
	size_t int_size = N * sizeof(int);
	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	cudaMemset(cudaArrays.grid_boids, 0, int_size);
	cudaMemset(cudaArrays.grid_cells, 0, int_size);
	cudaMemset(cudaArrays.grid_starts, -1, grid_size);
	cudaMemset(cudaArrays.grid_ends, -1, grid_size);	

	const int max_threads = 1024;
	int blocks_per_grid = (N + max_threads - 1) / max_threads;

	glm::mat3* models;
	cudaGraphicsMapResources(1, &cudaVBO, 0);
	cudaGraphicsResourceGetMappedPointer((void**)&models, NULL, cudaVBO);

	calculateGridKernel << <blocks_per_grid, max_threads >> > (cudaArrays);
	thrust::sort_by_key(thrust::device, cudaArrays.grid_cells, cudaArrays.grid_cells + N, cudaArrays.grid_boids);
	calculateGridStartsKernel << <blocks_per_grid, max_threads >> > (cudaArrays);
	calculateBoidsKernel << <blocks_per_grid, max_threads >> > (cudaArrays, shoal->params, deltaTime, models);

	cudaGraphicsUnmapResources(1, &cudaVBO, 0);
}
