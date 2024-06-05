#pragma once
#include <ray_tracing/embree_interface.h>
#include <rendering/reservoir.h>
#include <rendering/shading.h>
#include <scene/scene.h>
#include <ui/draw.h>
#include <utils/config.h>


// Light samplers
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color);
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color);
void sampleDiskLight(const DiskLight& diskLight, glm::vec3& position, glm::vec3& color);

// ReSTIR per-pixel canonical samples
Reservoir genCanonicalSamples(const Scene& scene, const EmbreeInterface& embreeInterface, const Features& features, const RayHit& rayHit);
