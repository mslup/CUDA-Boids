#include "framework.h"

__global__ void kernel(int* c, const int* a, const int* b)
{
	int i = threadIdx.x;
	c[i] = a[i] + b[i];
}

cudaError_t deployCuda(int* c, const int* a, const int* b, unsigned int size)
{
	int* dev_a = 0;
	int* dev_b = 0;
	int* dev_c = 0;
	cudaError_t cudaStatus;

	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
		goto Error;
	}

	// Allocate GPU buffers for three vectors (two input, one output)    .
	cudaStatus = cudaMalloc((void**)&dev_c, size * sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	cudaStatus = cudaMalloc((void**)&dev_b, size * sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc failed!");
		goto Error;
	}

	// Copy input vectors from host memory to GPU buffers.
	cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

	cudaStatus = cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

	// Launch a kernel on the GPU with one thread for each element.
	kernel << <1, size >> > (dev_c, dev_a, dev_b);

	// Check for any errors launching the kernel
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
		goto Error;
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
		goto Error;
	}

	// Copy output vector from GPU buffer to host memory.
	cudaStatus = cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy failed!");
		goto Error;
	}

Error:
	cudaFree(dev_c);
	cudaFree(dev_a);
	cudaFree(dev_b);

	return cudaStatus;
}

__device__ glm::vec2 apply_boid_rules(glm::vec2* pos, glm::vec2* vel, int i, double d)
{
	return glm::vec2(0, d);

	float visibility_radius = 1e-1;
	float s = 0.01;//3e-2;
	float a = 0.1;//8e-2;
	float c = 0.01;//9e-2;

	glm::vec2 separation_component(0, 0);
	glm::vec2 velocity_sum(0, 0);
	glm::vec2 position_sum(0, 0);
	int neighbors = 0;

	for (int j = 0; j < N; ++j)
	{
		float len = glm::length(pos[i] - pos[j]);
		if (i != j && len < visibility_radius)
		{
			separation_component += pos[i] - pos[j];
			velocity_sum += vel[j];
			position_sum += pos[j];

			neighbors++;
		}
	}

	glm::vec2 alignment_component;
	glm::vec2 cohesion_component;

	if (neighbors == 0)
		return glm::vec2(0, 0);

	velocity_sum /= neighbors;
	position_sum /= neighbors;
	alignment_component = velocity_sum - vel[i];
	cohesion_component = position_sum - pos[i];

	return glm::vec2(0, d);//(float)d* (s * separation_component + a * alignment_component + c * cohesion_component);
}

__global__ void calculateBoidsKernel(glm::vec2* pos,
	glm::vec2* vel, glm::vec2* pos_bb, glm::vec2* vel_bb, double d)
{
	int i = threadIdx.x;

	//vel[i] += apply_boid_rules(pos, vel, i, d);
	pos[i] += (float)d * vel[i];

	pos_bb[i] = pos[i];
	vel_bb[i] = vel[i];

}

__global__ void calculateModelKernel(glm::mat3* models, glm::vec2* pos, glm::vec2* vel)
{
	int i = threadIdx.x;

	// calculate model matrix
	glm::vec2 v = glm::normalize(vel[i]);
	glm::vec2 vT = glm::vec2(v.y, -v.x);
	models[i] = glm::mat3(glm::vec3(v, 0), glm::vec3(vT, 0), glm::vec3(pos[i], 1.0f));
}