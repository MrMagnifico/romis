# Software ReSTIR
A C++ implementation of [Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting](https://dl.acm.org/doi/abs/10.1145/3386569.3392481) on top of a Whitted-style ray tracer. Developed for the implementation component for the course IN4310 Seminar Computer Graphics.

## Build Instructions
As this project uses CMake, simply use your favourite CLI/IDE/code editor and compile the `SeminarImpl` target. A compiler with OpenMP 2.0+ support is required. All required libraries and resources are provided in this repository.

## Background
The original ReSTIR is an algorithm for accelerating light sampling in direct illumination computation by performing spatial and temporal reuse of samples using weighted reservoir sampling. This implementation aims to provide a playground to fiddle with the algorithm's parameters and steps in order to demonstrate their effects both in isolation and in tandem. For further specifics, see the report provided in the root directory.

This implementation makes use of the final project framework from the course CSE2215 Computer Graphics.

## Ray Tracer Specifics
The underlying ray tracer is a simple Whitted-style ray tracer that utilised a Phong shading model with diffuse and specular components. A singular bottom-level-style BVH is constructed on a per scene basis to accelerate ray tracing.
