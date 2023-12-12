#version 330 core

layout(location = 0) in vec2 aVertex;
layout(location = 1) in mat3 aModelMatrix;

out vec3 myColor;

uniform mat4 uProjMatrix;

void main()
{
	gl_Position = uProjMatrix * vec4((aModelMatrix * vec3(aVertex, 1)).xy, -1.0, 1.0);//
}