#include "framework.h"

Window::Window(Application *parent)
{
	app = parent;

	width = 800;
	height = 800;

	lastX = width / 2;
	lastY = height / 2;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

	wndptr = glfwCreateWindow(width, height, "Shoal of fish", NULL, NULL);
	if (wndptr == NULL)
	{
		std::cout << "Failed to create a window\n";
		glfwTerminate();
	}
	glfwMakeContextCurrent(wndptr);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initalize GLAD" << std::endl;
	}

	glViewport(0, 0, width, height);

	glfwSetInputMode(wndptr, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	glfwSetWindowUserPointer(wndptr, this);
	glfwSetFramebufferSizeCallback(wndptr, 
		[](GLFWwindow* window, int width, int height)
		{
			Window& wnd = *(Window*)glfwGetWindowUserPointer(window);
			wnd.width = width;
			wnd.height = height;

			glViewport(0, 0, width, height);
		});

	glfwSetCursorPosCallback(wndptr,
		[](GLFWwindow *window, double xposIn, double yposIn)
		{
			Window& wnd = *(Window*)glfwGetWindowUserPointer(window);

			float xpos = static_cast<float>(xposIn);
			float ypos = static_cast<float>(yposIn);

			if (wnd.firstMouse)
			{
				wnd.lastX = xpos;
				wnd.lastY = ypos;
				wnd.firstMouse = false;
			}

			float xoffset = xpos - wnd.lastX;
			float yoffset = wnd.lastY - ypos; // reversed since y-coordinates go from bottom to top
			wnd.lastX = xpos;
			wnd.lastY = ypos;

			wnd.app->updateCamera(xoffset, yoffset);
		});

	glfwSetScrollCallback(wndptr, 
		[](GLFWwindow* window, double xoffset, double yoffset)
		{
			Window& wnd = *(Window*)glfwGetWindowUserPointer(window);
			wnd.app->updateCameraZoom((float)yoffset);
		});

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

	ImGui_ImplGlfw_InitForOpenGL(wndptr, true);
	ImGui_ImplOpenGL3_Init();
}


void Window::processInput()
{
	if (glfwGetKey(wndptr, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(wndptr, true);

	//float cameraSpeed = static_cast<float>(2.5 * deltaTime);

	int cameraPosKeys[8] = {
		GLFW_KEY_W,
		GLFW_KEY_S,
		GLFW_KEY_A,
		GLFW_KEY_D,
		GLFW_KEY_SPACE,
		GLFW_KEY_LEFT_SHIFT,
		GLFW_KEY_Q,
		GLFW_KEY_E
	};

	for (int i = 0; i < 8; i++)
	{
		if (glfwGetKey(wndptr, cameraPosKeys[i]) == GLFW_PRESS)
			app->updateCameraPos(cameraPosKeys[i]);
	}
}