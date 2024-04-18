#pragma once

#include <framework/trackball.h>

#include <scene/scene.h>
#include <rendering/reservoir.h>
#include <rendering/screen.h>


// Common
ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, const Screen& screen, const Features& features);
glm::vec3 finalShading(const Reservoir& reservoir, const Ray& primaryRay, const BvhInterface& bvh, const Features& features);

// ReSTIR-specific
void spatialReuse(ReservoirGrid& reservoirGrid, const BvhInterface& bvh, const Screen& screen, const Features& features);
void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid, const BvhInterface& bvh,
                   Screen& screen, const Features& features);

// R-MIS-specific
float generalisedBalanceHeuristic(const LightSample& sample, const std::vector<Reservoir>& allPixels,
                                  const Ray& primaryRay, const HitInfo& primaryHitInfo,
                                  const Features& features);

// R-OMIS-specific
float arbitraryUnbiasedContributionWeightReciprocal(const LightSample& sample, const Reservoir& pixel, const Scene& scene,
                                                    size_t sampleIdx,
                                                    const Features& features);

