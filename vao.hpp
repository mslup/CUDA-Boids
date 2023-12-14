#include "framework.h"

class VAO
{
public:
	unsigned int boidVAO;
	unsigned int boidVBO;
	unsigned int boidEBO;
	unsigned int cubeVAO;
	unsigned int cubeVBO;
	unsigned int cubeEBO;
	unsigned int modelVBO;
	struct cudaGraphicsResource* cudaVBO;

};