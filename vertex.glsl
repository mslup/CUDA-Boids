#version 330 core

layout(location = 0) in vec3 aVertex;
layout(location = 1) in mat4 model;

out vec3 myColor;

uniform mat4 view;
uniform mat4 projection;

void main()
{
	//gl_Position = uProjMatrix * vec4((aModelMatrix * vec3(aVertex, 1)).xy, -0.7, 1.0);//
	//gl_Position = uProjMatrix * aModelMatrix * vec4(aVertex, 1.0);
	//gl_Position = uProjMatrix * vec4(aModelMatrix * vec4(aVertex, 1));
	gl_Position = projection * view * model * vec4(aVertex, 1);
}