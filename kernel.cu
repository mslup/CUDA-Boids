#include "framework.h"
#include <ctime>
#include <cstdlib>

int main()
{
	GLFWwindow* tmpwindow = initWindow();
	cpu_shoal* shoal = new cpu_shoal();

	srand(time(NULL));

	glm::vec2 center(0.0f, 0.0f);
	float height = 0.04f;
	float width = 0.02f;
	float vertices[] = {
		center.x + height / 2, center.y,
		center.x - height / 2, center.y - width / 2,
		center.x - height / 2, center.y + width / 2
	};

	// buffer objects
	unsigned int vertexVBO, VAO, modelVBO;
	{
		glGenBuffers(1, &vertexVBO);
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

		glGenBuffers(1, &modelVBO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), shoal->model, GL_DYNAMIC_DRAW);

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

	Shader shader("./vertex.glsl", "./fragment.glsl");
	shader.use();
	shader.setFloat3("boidColor", 0.9f, 0.5f, 0.0f);

	while (!glfwWindowShouldClose(tmpwindow))
	{
		processInput(tmpwindow);
		
		shoal->update_boids();

		// Render scene
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glBindVertexArray(VAO);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, N);

		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(shoal->model), &(shoal->model)[0], GL_DYNAMIC_DRAW);

		showError();

		glfwSwapBuffers(tmpwindow);
		glfwPollEvents();
	}

	glfwTerminate();
	delete shoal;

	return 0;
}

