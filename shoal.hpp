#pragma once
#include "framework.h"

class Shoal
{
public:

	constexpr static int vertexCount = 12;

	constexpr static float MIN_SEP_FACTOR = 0;
	constexpr static float MAX_SEP_FACTOR = 1e-1 / 2;
	constexpr static float SEP_DIVISOR = 10 / MAX_SEP_FACTOR;
	constexpr static float MIN_ALN_FACTOR = 0;
	constexpr static float MAX_ALN_FACTOR = 10;
	constexpr static float MIN_COH_FACTOR = 0;
	constexpr static float MAX_COH_FACTOR = 10;
	constexpr static float MIN_MINSPEED = 1e-3;
	constexpr static float MAX_MAXSPEED = 1;
	constexpr static float MAX_TURN_FACTOR = 30;
	constexpr static float MIN_TURN_FACTOR = 1;
	constexpr static float MIN_R = 0.04f;
	constexpr static float MAX_R = 0.5f;
	constexpr static float MIN_GRID_R = 2 * MIN_R;

	struct behaviourParamsStruct {
		float sep_factor, aln_factor, coh_factor,
			margin, turn_factor, 
			max_speed, min_speed, 
			visibility_radius;
		
	};
	struct behaviourParamsStruct behaviourParams;

	struct renderParamsStruct {
		float height, width;
		float vertices[vertexCount];
		unsigned int indices[vertexCount];
	};
	struct renderParamsStruct renderParams;

	glm::vec3 positions[Application::N];
	glm::vec3 positions_bb[Application::N];
	glm::vec3 velocities[Application::N];
	glm::vec3 velocities_bb[Application::N];
	glm::mat4 models[Application::N];

	Shoal();

	void init_positions();
	void init_velocities();

	glm::mat4 calculate_rotate(glm::vec3 pos, glm::vec3 vel);

	void update_boids_cpu(double d);
	void update_boids_gpu(cudaArrays soa, double d, struct cudaGraphicsResource* cudaVBO);
	void calculate_all_models();

	//const int N = 5000;

private:
	glm::vec3 apply_boid_rules(int i, float d);
	glm::vec3 turn_from_wall(glm::vec3 pos, glm::vec3 vel, float d);
	glm::vec3 speed_limit(glm::vec3 vel);
};

