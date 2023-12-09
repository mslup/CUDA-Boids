#version 330 core

layout(location = 0) in vec2 aVertex;
layout(location = 1) in mat3 aModelMatrix;

out vec3 myColor;

void main()
{
	gl_Position = vec4((aModelMatrix * vec3(aVertex, 1)).xy, 0.0, 1.0);//
	//gl_Position = vec4(aVertex, 0, 1);

	//if (abs(dot((aModelMatrix * vec3(1, 0, 0)).xy, vec2(1,0))) < 1e-3)
		myColor = vec3(0.5, 0.5, 0);
	//else
	//	myColor = vec3(0, 0.5, 0.5);
}