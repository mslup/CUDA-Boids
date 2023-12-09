#version 330 core

in vec3 myColor;

out vec4 FragColor;

uniform vec3 boidColor;

void main()
{
	//FragColor = vec4(boidColor, 1.0);//vec4(0.7, 0.4, 0.0, 1);
	FragColor = vec4(myColor, 1.0);
}
