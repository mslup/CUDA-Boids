#version 330 core

out vec4 FragColor;

uniform vec3 boidColor;

void main()
{
	FragColor = vec4(boidColor, 1.0);//vec4(0.7, 0.4, 0.0, 1);
}
