#version 330 core

layout(location = 0) in vec3 aVertex;
layout(location = 1) in mat4 model;

out vec3 color;

uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * model * vec4(aVertex, 1);

	color = (vec3(model[3][0], model[3][1], model[3][2]) + 2) / 4;
}