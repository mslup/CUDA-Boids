# Shoal of fish simulation project

## Overview

This project simulates the behavior of a shoal of fish, implemented for Graphic Processors in Computational Applications at the Faculty of Mathematics and Information Science at WUT. The simulation runs on GPU on default. There's a CPU version of the project available for comparison. The project is written in C++ and utilizes CUDA for computation and OpenGL for graphics rendering.

## Fish behavior 

The default number of fish in the shoal is 10,000. The behaviour of each fish is based on the [boid model](https://en.wikipedia.org/wiki/Boids) developed by Craig Reynolds in 1986.
- **Separation:** Influences a fish's velocity vector component based on the sum of differences in positions from its neighbors.
- **Alignment:** Influences a fish's velocity vector component based on the difference between its current velocity and the average velocity of its neighbors.
- **Cohesion:** Influences a fish's velocity vector component based on the difference between its current position and the average position of its neighbors.
- **Visibility Radius:** Radius of the sphere within which neighbors are considered.
- **Maximum Speed, Minimum Speed:** Limits on fish speed.
- **Turn from Walls:** Coefficient by which fish are drawn from the outside towards the center of the aquarium.
- **Margin:** Distance from the aquarium wall where the force turning fish away from the wall begins.

## Grid System

For optimalization reasons, the aquarium is divided into cubes of size 2R x 2R x 2R, where R is the visibility radius. Each fish is assigned a cell index, and these pairs (cell index, fish index) are sorted based on the cell index. To find neighbors, a maximum of 8 cells is considered: the one containing the fish and 7 neighboring cells based on the fish's position within its cell.

## Kernels

In each animation frame, the following kernels are launched:

- `calculateGridKernel`: Calculates the cell index for each fish (pairs are sorted using the Thrust library).
- `calculateGridStartsKernel`: Determines the beginnings and ends of sequences of cells.
- `calculateBoidsKernel`: Determines new velocities and positions, saved in the backbuffer, then copied to regular velocity and position arrays after synchronization. Model matrices (responsible for translation and rotation) for each fish are also determined.

## Demo

![demo1](https://github.com/mslup/CUDA-Boids/assets/132074948/5c5f506e-e93b-4b07-989e-7b82dbae4a67)
![demo2](https://github.com/mslup/CUDA-Boids/assets/132074948/52204f20-03cf-4261-9580-43b9f2c5d678)




