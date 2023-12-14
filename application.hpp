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

	constexpr static int N = BOID_COUNT;
	constexpr static int WIDTH = 800;
	constexpr static int HEIGHT = 800;

	constexpr static float LEFT_WALL = -2;
	constexpr static float DOWN_WALL = -2;
	constexpr static float BACK_WALL = -2;
	constexpr static float WORLD_WIDTH = -2 * LEFT_WALL;

	void processMouseMovement(float xoffset, float yoffset);
	void updateCameraZoom(float yoffset);
	void processKeyboard(int);

private:
	constexpr static float X = -LEFT_WALL;
	constexpr static float Y = -DOWN_WALL;
	constexpr static float Z = -BACK_WALL;

	const float cubeVertices[3 * 8] = {
		 X,  Y,  Z,
		-X,  Y,  Z,
		-X, -Y,  Z,
		 X, -Y,  Z,
		 X,  Y, -Z,
		-X,  Y, -Z,
		-X, -Y, -Z,
		 X, -Y, -Z
	};

	const unsigned int cubeIndices[2 * 12] = {
		0, 1,
		1, 2,
		2, 3,
		3, 0,
		4, 5,
		5, 6,
		6, 7,
		7, 4,
		1, 5,
		2, 6,
		3, 7,
		0, 4
	};

	Shoal* shoal;
	Window* window;
	VAO* vao;
	Shader* boidShader;
	Shader* cubeShader;
	Camera* camera;

	float x, y, z;

	bool pause = false;
	bool freeCamera = false;
	double deltaTime;

	struct cudaArrays soa;

	const glm::vec3 backColor = glm::vec3(65, 55, 46);
	const glm::vec3 cubeColor = glm::vec3(213, 189, 175);
	const glm::vec3 cubeColorCpu = glm::vec3(114, 219, 219);
	const size_t mat_size = N * sizeof(glm::mat4);
	const size_t vec_size = N * sizeof(glm::vec3);
	const size_t int_size = N * sizeof(int);

	void imGuiFrame(int);
	void create_buffer_objects();
	void update();
	void initValues();
};