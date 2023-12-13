#version 330 core

layout(location = 0) in vec3 aVertex;
layout(location = 1) in mat4 model;

out vec3 myColor;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 boidColor;

void main()
{
	gl_Position = projection * view * model * vec4(aVertex, 1);

	//if (abs(dot((model * vec4(1, 0, 0, 0)).xyz, vec3(0, 1, 0)) - 1) < 1e-3)
	//	myColor = vec3(1, 1, 1);//65.0 / 255, 55.0 / 255, 46.0 / 255);//1, 1, 1);
	//else
	myColor = (vec3(model[3][0], model[3][1], model[3][2]) + 1) / 2;
}