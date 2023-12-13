#include "framework.h"

class VAO
{
public:
	unsigned int VAO;
	unsigned int vertexVBO;
	unsigned int vertexEBO;
	unsigned int modelVBO;
	struct cudaGraphicsResource* cudaVBO;

};