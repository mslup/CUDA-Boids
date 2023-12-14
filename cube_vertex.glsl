#version 330 core

layout(location = 0) in vec3 aVertex;

out vec3 color;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 cubeColor;

void main()
{
	gl_Position = projection * view * vec4(aVertex, 1);

	color = cubeColor;
}