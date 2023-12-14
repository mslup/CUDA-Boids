#include "framework.h"
#include <ctime>
#include <cstdlib>

void checkCudaError() {
	cudaError_t error = cudaGetLastError();
	if (error != cudaSuccess) {
		std::cerr << "CUDA error: " << cudaGetErrorString(error) << std::endl;
	}
}

void callKernels(int blocks_per_grid, int max_threads, double deltaTime,
	glm::mat4* models, Shoal* shoal, struct cudaArrays soa, float x, float y, float z)
{
	calculateGridKernel << <blocks_per_grid, max_threads >> > (soa, shoal->behaviourParams.visibility_radius);
	gpuErrchk(cudaPeekAtLastError());
	gpuErrchk(cudaDeviceSynchronize());

	thrust::sort_by_key(thrust::device, soa.grid_cells, soa.grid_cells + Application::N, soa.grid_boids);
	gpuErrchk(cudaPeekAtLastError());
	gpuErrchk(cudaDeviceSynchronize());

	calculateGridStartsKernel << <blocks_per_grid, max_threads >> > (soa);
	gpuErrchk(cudaPeekAtLastError());
	gpuErrchk(cudaDeviceSynchronize());

	calculateBoidsKernel << <blocks_per_grid, max_threads >> > (soa, shoal->behaviourParams, 
		deltaTime, models);
	gpuErrchk(cudaPeekAtLastError());
	gpuErrchk(cudaDeviceSynchronize());

}

int main()
{
	Application* app = new Application();
	app->run();

	return 0;
}



