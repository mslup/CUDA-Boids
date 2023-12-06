#include "framework.h"
#include <ctime>
#include <cstdlib>

int main()
{
	srand(time(NULL));

	initWindow();
	cpu_shoal* shoal = new cpu_shoal();


	create_buffer_objects(shoal);

	Shader shader("./vertex.glsl", "./fragment.glsl");
	shader.use();
	shader.setFloat3("boidColor", 0.9f, 0.5f, 0.0f);

	render(shoal);

	
	delete shoal;

	return 0;
}

