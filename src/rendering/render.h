#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <Eigen/Dense>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

#include <framework/ray.h>

#include <rendering/reservoir.h>

#include <optional>
#include <vector>

// Forward declarations.
struct Scene;
class Screen;
class Trackball;
class BvhInterface;
struct Features;


// All rendering modes on offer
ReservoirGrid renderReSTIR(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                           const Scene& scene, const Trackball& camera,
                           const EmbreeInterface& embreeInterface, Screen& screen,
                           const Features& features);
void renderRMIS(const Scene& scene, const Trackball& camera, const EmbreeInterface& embreeInterface, Screen& screen, const Features& features);
void renderROMIS(const Scene& scene, const Trackball& camera, const EmbreeInterface& embreeInterface, Screen& screen, const Features& features);

// Entry point to ray-tracing rendering modes
std::optional<ReservoirGrid> renderRayTraced(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                                             const Scene& scene, const Trackball& camera,
                                             const EmbreeInterface& embreeInterface, Screen& screen,
                                             const Features& features);
