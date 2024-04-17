#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

#include <framework/ray.h>

#include <rendering/reservoir.h>

// Forward declarations.
struct Scene;
class Screen;
class Trackball;
class BvhInterface;
struct Features;

// Common
ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, const Screen& screen, const Features& features);
glm::vec3 finalShading(const Reservoir& reservoir, const Ray& primaryRay, const BvhInterface& bvh, const Features& features);

// ReSTIR-specific
void spatialReuse(ReservoirGrid& reservoirGrid, const BvhInterface& bvh, const Screen& screen, const Features& features);
void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid, const BvhInterface& bvh,
                   Screen& screen, const Features& features);

// Render using ReSTIR
ReservoirGrid renderReSTIR(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                           const Scene& scene, const Trackball& camera,
                           const BvhInterface& bvh, Screen& screen,
                           const Features& features);

// Render using R-MIS
void renderRMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features);
