#include "framework.h"

Application::Application()
{
	shoal = new Shoal();
	vao = new VAO();

#ifndef CPU
	cudaSetDevice(0);
#endif

	srand(time(NULL));
	window = new Window();

	create_buffer_objects();

	Shader shader("./vertex.glsl", "./fragment.glsl");
	shader.use();

#ifdef CPU
	shader.setFloat3("boidColor", 0.2f, 0.7f, 0.4f);
#else
	shader.setFloat3("boidColor",
		boidColor.r / 255.0f,
		boidColor.g / 255.0f,
		boidColor.b / 255.0f);

	glm::mat4 view = glm::mat4(1);
	view = glm::translate(view, glm::vec3(0, 0, -3));
	shader.setMat4("view", view);

	glm::mat4 proj = glm::mat4(1.0);
	proj = glm::perspective(glm::radians(45.0f), (float)window->width / (float)window->height, 0.0f, 100.0f);
	shader.setMat4("projection", proj);

	size_t density = (int)glm::ceil(WORLD_WIDTH / GRID_R);
	size_t grid_size = density * density * sizeof(int);

	cudaMalloc(&soa.models, mat_size);
	cudaMalloc(&soa.positions, vec_size);
	cudaMalloc(&soa.velocities, vec_size);
	cudaMalloc(&soa.positions_bb, vec_size);
	cudaMalloc(&soa.velocities_bb, vec_size);
	cudaMalloc(&soa.grid_cells, int_size);
	cudaMalloc(&soa.grid_boids, int_size);
	cudaMalloc(&soa.grid_starts, grid_size);
	cudaMalloc(&soa.grid_ends, grid_size);

	cudaMemcpy(soa.positions, shoal->positions, vec_size, cudaMemcpyHostToDevice);
	cudaMemcpy(soa.velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice);
#endif
}

void Application::initValues()
{

}

void Application::run()
{
	glfwSwapInterval(0);

	int num_frames = 0;
	int fps = 0;
	double previousTime = glfwGetTime();
	double previousFpsTime = previousTime;

	while (!glfwWindowShouldClose(window->wndptr))
	{
		window->processInput();
		imGuiFrame();

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

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

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glBindVertexArray(vao->VAO);
		glBindBuffer(GL_ARRAY_BUFFER, vao->modelVBO);
		glBufferData(GL_ARRAY_BUFFER, mat_size, 0, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		cudaGraphicsGLRegisterBuffer(&vao->cudaVBO, vao->modelVBO, cudaGraphicsMapFlagsWriteDiscard);

		if (!pause)
			update(deltaTime);

		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window->wndptr);
		glfwPollEvents();
	}
}

Application::~Application()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();

	delete shoal;
	cudaFree(soa.models);
	cudaFree(soa.positions);
	cudaFree(soa.positions_bb);
	cudaFree(soa.velocities);
	cudaFree(soa.velocities_bb);
	cudaFree(soa.grid_cells);
	cudaFree(soa.grid_boids);
	cudaFree(soa.grid_starts);
	cudaFree(soa.grid_ends);

	if (cudaDeviceReset() != cudaSuccess) {
		fprintf(stderr, "cudaDeviceReset failed!");
	}
}


void Application::imGuiFrame()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	if (ImGui::Button("Pause"))
	{
		pause = !pause;
	}

	ImGui::SliderFloat("cohesion", &shoal->params.c, 0.0f, 0.5f);
	ImGui::SliderFloat("separation", &shoal->params.s, 0.0f, 0.01f);
	ImGui::SliderFloat("alignment", &shoal->params.a, 0.0f, 0.5f);
	ImGui::SliderFloat("max_speed", &shoal->params.max_speed, shoal->params.min_speed, 1.0f);
	ImGui::SliderFloat("min_speed", &shoal->params.min_speed, 0.0f, shoal->params.max_speed);
	ImGui::SliderFloat("visbility_radius", &shoal->params.visibility_radius, 0.0f, 0.5f);

}

void Application::update(double deltaTime)
{
#ifdef CPU
	shoal->update_boids_cpu(deltaTime);
#else
	shoal->update_boids_gpu(soa, deltaTime, vao->cudaVBO);
#endif
}

void Application::create_buffer_objects()
{
	glGenBuffers(1, &vao->vertexVBO);
	glGenVertexArrays(1, &vao->VAO);
	glBindVertexArray(vao->VAO);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vao->vertexVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->params.vertices), shoal->params.vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

	glGenBuffers(1, &vao->modelVBO);
	glBindBuffer(GL_ARRAY_BUFFER, vao->modelVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->models), shoal->models, GL_DYNAMIC_DRAW);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)0);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4)));
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(2 * sizeof(glm::vec4)));
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(3 * sizeof(glm::vec4)));

	glVertexAttribDivisor(1, 1);
	glVertexAttribDivisor(2, 1);
	glVertexAttribDivisor(3, 1);
	glVertexAttribDivisor(4, 1);
}