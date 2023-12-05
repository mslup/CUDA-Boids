#version 330 core

layout(location = 0) in vec2 aVertex;
layout(location = 1) in mat3 aModelMatrix;

void main()
{
	gl_Position = vec4((aModelMatrix * vec3(aVertex, 1)).xy, 0.0, 1.0);//
	//gl_Position = vec4(aVertex, 0, 1);
}