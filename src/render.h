#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <framework/ray.h>
#include <reservoir.h>

// Forward declarations.
struct Scene;
class Screen;
class Trackball;
class BvhInterface;
struct Features;

ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features);

// Main rendering function.
ReservoirGrid renderRayTracing(const Scene& scene, const Trackball& camera,
                               const BvhInterface& bvh, const ReservoirGrid& previousFrameGrid, Screen& screen,
                               const Features& features);
