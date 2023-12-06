#include "framework.h"

GLFWwindow* window;

unsigned int VBO, VAO;
unsigned int modelVBO;

//__global__ void kernel_tmp(glm::mat3* models);

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

void showError()
{
	GLenum error = glGetError();
	if (error != GL_NO_ERROR) {
		std::cerr << "OpenGL error: " << error << std::endl;
	}
}

GLFWwindow* initWindow()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

	window = glfwCreateWindow(WIDTH, HEIGHT, "Shoal of fish", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create a window\n";
		glfwTerminate();
		//return -1;
		return nullptr;
	}
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initalize GLAD" << std::endl;
		return nullptr;
	}

	glViewport(0, 0, WIDTH, HEIGHT);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	return window;
}


void create_buffer_objects(cpu_shoal* shoal)
{
	glGenBuffers(1, &VBO);
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->vertices), shoal->vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

	glGenBuffers(1, &modelVBO);
	glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), shoal->model, GL_STATIC_DRAW);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::mat3), (void*)0);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(glm::mat3), (void*)(sizeof(glm::vec3)));
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(glm::mat3), (void*)(2 * sizeof(glm::vec3)));

	glVertexAttribDivisor(1, 1);
	glVertexAttribDivisor(2, 1);
	glVertexAttribDivisor(3, 1);
}

void render(cpu_shoal *shoal)
{
//	Shader shader("./vertex.glsl", "./fragment.glsl");
//	shader.use();
//	shader.setFloat3("boidColor", 0.9f, 0.5f, 0.0f);
//
//	while (!glfwWindowShouldClose(window))
//	{
//		processInput(window);
//
//		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//		glClear(GL_COLOR_BUFFER_BIT);
//
//		glBindVertexArray(VAO);
//		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);
//
//		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
//
//#ifdef GPU
//		//glm::mat3* model;
//		//cudaMalloc(&model, N * sizeof(glm::mat3));
//
//		//kernel_tmp << <1, N >> > (model);
//
//		//size_t size = N * sizeof(glm::mat3);
//		//glm::mat3* host_model = (glm::mat3*)malloc(size);
//
//		//cudaMemcpy(host_model, model, size, cudaMemcpyHostToDevice);
//		//glBufferData(GL_ARRAY_BUFFER, size, host_model, GL_DYNAMIC_DRAW);
//
//		//free(host_model);
//		//cudaFree(model);
//#else
//		shoal->update_boids(d);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), &(shoal->model)[0], GL_DYNAMIC_DRAW);
//#endif
//
//		showError();
//
//		glfwSwapBuffers(window);
//		glfwPollEvents();
//	}
//
//	glfwTerminate();
}

//__global__ void kernel_tmp(glm::mat3* models)
//{
//	int i = threadIdx.x;
//	glm::vec2 pos = glm::vec2(i * 1.0f / N, 0);
//
//	glm::vec2 v = glm::vec2(0, 1);
//	glm::vec2 vT = glm::vec2(v.y, -v.x);
//	models[i] = glm::mat3(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(pos, 1.0f));
//}