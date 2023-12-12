#include "framework.h"

class Window
{
public:
	int width, height;

	Window();
	GLFWwindow* wndptr;

	void processInput();

private:
};