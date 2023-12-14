#include "framework.h"

Application::Application()
{
	x = y = z = 0;

	shoal = new Shoal();
	vao = new VAO();
	camera = new Camera();

#ifndef CPU
	gpuErrchk(cudaSetDevice(0));
#endif

	srand(time(NULL));
	window = new Window(this);

	create_buffer_objects();

	boidShader = new Shader("./boid_vertex.glsl", "./fragment.glsl");
	cubeShader = new Shader("./cube_vertex.glsl", "./fragment.glsl");

	cubeShader->use();
#ifdef CPU
	cubeShader->setFloat3("cubeColor",
		cubeColorCpu.r / 255.0f,
		cubeColorCpu.g / 255.0f,
		cubeColorCpu.b / 255.0f);
#else // GPU
	cubeShader->setFloat3("cubeColor",
		cubeColor.r / 255.0f,
		cubeColor.g / 255.0f,
		cubeColor.b / 255.0f);

	size_t max_density = (int)glm::ceil(WORLD_WIDTH / Shoal::MIN_GRID_R);
	size_t max_grid_size = max_density * max_density * max_density * sizeof(int);

	gpuErrchk(cudaMalloc(&soa.models, mat_size));
	gpuErrchk(cudaMalloc(&soa.positions, vec_size));
	gpuErrchk(cudaMalloc(&soa.velocities, vec_size));
	gpuErrchk(cudaMalloc(&soa.positions_bb, vec_size));
	gpuErrchk(cudaMalloc(&soa.velocities_bb, vec_size));
	gpuErrchk(cudaMalloc(&soa.grid_cells, int_size));
	gpuErrchk(cudaMalloc(&soa.grid_boids, int_size));
	gpuErrchk(cudaMalloc(&soa.grid_starts, max_grid_size));
	gpuErrchk(cudaMalloc(&soa.grid_ends, max_grid_size));

	gpuErrchk(cudaMemcpy(soa.positions, shoal->positions, vec_size, cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(soa.velocities, shoal->velocities, vec_size, cudaMemcpyHostToDevice));
#endif
}

Application::~Application()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();

	delete shoal;
	delete vao;
	delete camera;
	delete window;
	delete boidShader;
	delete cubeShader;

#ifndef CPU
	gpuErrchk(cudaFree(soa.models));
	gpuErrchk(cudaFree(soa.positions));
	gpuErrchk(cudaFree(soa.positions_bb));
	gpuErrchk(cudaFree(soa.velocities));
	gpuErrchk(cudaFree(soa.velocities_bb));
	gpuErrchk(cudaFree(soa.grid_cells));
	gpuErrchk(cudaFree(soa.grid_boids));
	gpuErrchk(cudaFree(soa.grid_starts));
	gpuErrchk(cudaFree(soa.grid_ends));

	gpuErrchk(cudaDeviceReset());
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

		double currentTime = glfwGetTime();
		deltaTime = currentTime - previousTime;
		deltaTime = glm::min(deltaTime, 1.0 / 60.0);
		previousTime = currentTime;

		num_frames++;
		if (currentTime - previousFpsTime >= 1.0)
		{
			fps = num_frames;
			num_frames = 0;
			previousFpsTime += 1.0;
		}

		imGuiFrame(fps);

		glClearColor(
			backColor.r / 255.0f,
			backColor.g / 255.0f,
			backColor.b / 255.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 proj = camera->GetProjectionMatrix(window->width, window->height);
		glm::mat4 view = camera->GetViewMatrix();

		glBindVertexArray(vao->cubeVAO);
		cubeShader->use();
		cubeShader->setMat4("projection", proj);
		cubeShader->setMat4("view", view);
		glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, (void*)0);

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glBindVertexArray(vao->boidVAO);
		glBindBuffer(GL_ARRAY_BUFFER, vao->modelVBO);
#ifdef CPU
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->models), shoal->models, GL_DYNAMIC_DRAW);
#else // GPU
		glBufferData(GL_ARRAY_BUFFER, mat_size, 0, GL_DYNAMIC_DRAW);
#endif
		glBindBuffer(GL_ARRAY_BUFFER, 0);


		if (!pause)
			update();

		boidShader->use();
		boidShader->setMat4("projection", proj);
		boidShader->setMat4("view", view);

		glDrawElementsInstanced(GL_TRIANGLES, Shoal::vertexCount, GL_UNSIGNED_INT, 0, N);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window->wndptr);
		glfwPollEvents();
	}
}

void Application::imGuiFrame(int fps)
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	if (!ImGui::Begin("Menu", NULL, 0))
	{
		ImGui::End();
		return;
	}

	ImGui::PushItemWidth(-ImGui::GetWindowWidth() * 0.45f);
	//ImGui::PushItemWidth(ImGui::GetFontSize() * -6);

	ImGui::Text("%d fps", fps);

	if (ImGui::Button("Pause"))
	{
		pause = !pause;
	}

	if (ImGui::CollapsingHeader("Boid parameters"))
	{
		ImGui::SeparatorText("Neighbour behaviour");
		ImGui::SliderFloat("Cohesion", &shoal->behaviourParams.coh_factor,
			Shoal::MIN_COH_FACTOR, Shoal::MAX_COH_FACTOR);
		ImGui::SliderFloat("Separation", &shoal->behaviourParams.sep_factor,
			Shoal::MIN_SEP_FACTOR * Shoal::SEP_DIVISOR, Shoal::MAX_SEP_FACTOR * Shoal::SEP_DIVISOR);
		ImGui::SliderFloat("Alignment", &shoal->behaviourParams.aln_factor,
			Shoal::MIN_ALN_FACTOR, Shoal::MAX_ALN_FACTOR);
		ImGui::SliderFloat("Visibility radius", &shoal->behaviourParams.visibility_radius,
			Shoal::MIN_R, Shoal::MAX_R);

		ImGui::SeparatorText("Restrictions");
		ImGui::SliderFloat("Maximum speed", &shoal->behaviourParams.max_speed,
			shoal->behaviourParams.min_speed, Shoal::MAX_MAXSPEED);
		ImGui::SliderFloat("Minimum speed", &shoal->behaviourParams.min_speed,
			Shoal::MIN_MINSPEED, shoal->behaviourParams.max_speed);
		ImGui::SliderFloat("Turn from walls", &shoal->behaviourParams.turn_factor,
			Shoal::MIN_TURN_FACTOR, Shoal::MAX_TURN_FACTOR);
		ImGui::SliderFloat("Wall margin", &shoal->behaviourParams.margin,
			Shoal::MIN_MARGIN, Shoal::MAX_MARGIN);
	}

	if (ImGui::CollapsingHeader("Information"))
	{
		float grid_radius = 2 * shoal->behaviourParams.visibility_radius;
		int density = (int)glm::ceil(Application::WORLD_WIDTH / grid_radius);
		int grid_size = density * density * density;

		ImGui::Text("Number of boids: %d", N);

		ImGui::Text("Current grid density: %d", density);
		ImGui::Text("Number of cells: % d", grid_size);

#ifdef CPU
		ImGui::Text("Solution: CPU");
#else
#ifdef NAIVE
		ImGui::Text("Solution: GPU (naive)");
#else
		ImGui::Text("Solution: GPU (grid)");
#endif
#endif
	}

	if (ImGui::CollapsingHeader("Camera controls"))
	{
		ImGui::SeparatorText("Position");
		ImGui::BulletText("W - forward");
		ImGui::BulletText("S - backward");
		ImGui::BulletText("A - left");
		ImGui::BulletText("R - right");
		ImGui::BulletText("Space - up");
		ImGui::BulletText("LShift - down");

		ImGui::SeparatorText("Angles");
		ImGui::BulletText("Q - look left");
		ImGui::BulletText("E - look right");
		ImGui::BulletText("1 - look up");
		ImGui::BulletText("3 - look down");

		ImGui::SeparatorText("Misc");
		ImGui::BulletText("F - toggle mouse/keyboard");
	}
}

void Application::update()
{
#ifdef CPU
	shoal->update_boids_cpu(deltaTime);
#else
	shoal->update_boids_gpu(soa, deltaTime, vao->cudaVBO);
#endif
}

void Application::create_buffer_objects()
{
	// boids
	glGenVertexArrays(1, &vao->boidVAO);
	glGenBuffers(1, &vao->boidVBO);
	glBindVertexArray(vao->boidVAO);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vao->boidVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->renderParams.vertices), shoal->renderParams.vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	glGenBuffers(1, &vao->boidEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao->boidEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(shoal->renderParams.indices), shoal->renderParams.indices, GL_STATIC_DRAW);

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

	cudaGraphicsGLRegisterBuffer(&vao->cudaVBO, vao->modelVBO, cudaGraphicsMapFlagsWriteDiscard);

	// cube
	glGenVertexArrays(1, &vao->cubeVAO);
	glGenBuffers(1, &vao->cubeVBO);
	glBindVertexArray(vao->cubeVAO);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vao->cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	glGenBuffers(1, &vao->cubeEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao->cubeEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);
}

void Application::processMouseMovement(float xoffset, float yoffset)
{
	if (freeCamera)
		camera->Rotate(xoffset, yoffset, true);
}

void Application::updateCameraZoom(float yoffset)
{
	camera->ProcessMouseScroll(yoffset);
}

void Application::processKeyboard(int key)
{
	if (key == GLFW_KEY_F)
	{
		freeCamera = !freeCamera;
		if (freeCamera)
			glfwSetInputMode(window->wndptr, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		else
			glfwSetInputMode(window->wndptr, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		return;
	}

	if (freeCamera && (key == GLFW_KEY_Q ||
		key == GLFW_KEY_E ||
		key == GLFW_KEY_1 ||
		key == GLFW_KEY_3))
		return; 

	camera->ProcessKeyboard(key, deltaTime);
}