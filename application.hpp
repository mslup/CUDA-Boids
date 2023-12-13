#include "framework.h"

class Shoal;
class Shader;
class Window;
class VAO;
class Camera;

class Application 
{
public:
	Application();
	~Application();

	void run();
	
	constexpr static int N = 5000;
	
	void updateCameraAngles(float xoffset, float yoffset);
	void updateCameraZoom(float yoffset);
	void updateCameraPos(int);

private:
	Shoal* shoal;
	Window* window;
	VAO* vao;
	Shader* shader;
	Camera* camera;

	float x, y, z;

	bool pause = false;
	double deltaTime;

	struct cudaArrays soa;

	const glm::vec3 backColor = glm::vec3(65, 55, 46);//84, 65, 78);
	const glm::vec3 boidColor = glm::vec3(213, 189, 175);//167, 144, 165);
	const size_t mat_size = N * sizeof(glm::mat4);
	const size_t vec_size = N * sizeof(glm::vec3);
	const size_t int_size = N * sizeof(int);

	void imGuiFrame();
	void create_buffer_objects();
	void update();
	void initValues();
};