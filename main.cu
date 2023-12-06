#include "framework.h"
#include <ctime>
#include <cstdlib>

int main()
{

#ifndef CPU
	cudaSetDevice(0);
#endif

	srand(time(NULL));

	initWindow();
	cpu_shoal* shoal = new cpu_shoal();

	create_buffer_objects(shoal);

	Shader shader("./vertex.glsl", "./fragment.glsl");
	shader.use();
	shader.setFloat3("boidColor", 0.9f, 0.5f, 0.0f);

	double previousTime = glfwGetTime();
	while (!glfwWindowShouldClose(window))
	{
		double currentTime = glfwGetTime(); //TODO: platform
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

		processInput(window);

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);

#ifndef CPU
		size_t mat_size = N * sizeof(glm::mat3);
		size_t vec_size = N * sizeof(glm::vec2);

		glm::mat3* host_model = (glm::mat3*)malloc(mat_size);

		glm::mat3* model;
		cudaMalloc(&model, mat_size);
		glm::vec2* positions;
		cudaMalloc(&positions, vec_size);
		glm::vec2* velocities;
		cudaMalloc(&velocities, vec_size);
		glm::vec2* positions_bb;
		cudaMalloc(&positions_bb, vec_size);
		glm::vec2* velocities_bb;
		cudaMalloc(&velocities_bb, vec_size);

		cudaMemcpy(positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
		cudaMemcpy(velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);

		calculateBoidsKernel << <1, N >> > (positions, velocities, 
			positions_bb, velocities_bb, deltaTime);

		cudaMemcpy(positions, positions_bb, vec_size, cudaMemcpyDeviceToDevice);
		cudaMemcpy(velocities, velocities_bb, vec_size, cudaMemcpyDeviceToDevice);
		
		calculateModelKernel << <1, N >> > (model, positions, velocities);

		cudaMemcpy(shoal->positions, positions_bb, vec_size, cudaMemcpyDeviceToHost);
		cudaMemcpy(shoal->velocities, velocities_bb, vec_size, cudaMemcpyDeviceToHost);

		//std::cout << shoal->positions[0].x << std::endl;

		cudaMemcpy(host_model, model, mat_size, cudaMemcpyDeviceToHost);
		glBufferData(GL_ARRAY_BUFFER, mat_size, host_model, GL_DYNAMIC_DRAW);

		free(host_model);
		cudaFree(model);
		cudaFree(positions);
		cudaFree(positions_bb);
		cudaFree(velocities);
		cudaFree(velocities_bb);
#else
		shoal->update_boids();
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), &(shoal->model)[0], GL_DYNAMIC_DRAW);
#endif

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
		showError();

		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	glfwTerminate();
	
	delete shoal;

	return 0;
}

