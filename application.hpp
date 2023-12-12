#include "framework.h"

class Shoal;
class Window;
class VAO;

class Application 
{
public:
	Application();
	~Application();

	void run();
	
	constexpr static int N = 4096;

private:
	Shoal* shoal;
	Window* window;
	VAO* vao;

	bool pause = false;
	struct cudaArrays soa;

	const glm::vec3 backColor = glm::vec3(65, 55, 46);//84, 65, 78);
	const glm::vec3 boidColor = glm::vec3(213, 189, 175);//167, 144, 165);
	const size_t mat_size = N * sizeof(glm::mat3);
	const size_t vec_size = N * sizeof(glm::vec2);
	const size_t int_size = N * sizeof(int);

	void imGuiFrame();
	void create_buffer_objects();
	void update(double deltaTime);
	void initValues();
};