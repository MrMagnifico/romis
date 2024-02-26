#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <framework/ray.h>
#include <reservoir.h>

#include <optional>
#include <vector>

// Forward declarations.
struct Scene;
class Screen;
class Trackball;
class BvhInterface;
struct Features;

using GridMask = std::vector<std::vector<bool>>;

GridMask genCanonicalMask(std::shared_ptr<ReservoirGrid> previousFrameGrid, const Screen& screen, const Features& features);

ReservoirGrid genInitialSamples(uint32_t frameCount, std::optional<GridMask> generateCanonical,
                                const Scene& scene, const Trackball& camera, const BvhInterface& bvh, const Screen& screen, const Features& features);

void spatialReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& resampleGrid,
                  const BvhInterface& bvh, const Screen& screen, const Features& features);

void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid,
                   const BvhInterface& bvh, Screen& screen, const Features& features);

// Main rendering function.
ReservoirGrid renderRayTracing(std::shared_ptr<ReservoirGrid> previousFrameGrid, uint32_t frameCount,
                               const Scene& scene, const Trackball& camera,
                               const BvhInterface& bvh, Screen& screen,
                               const Features& features);
