#pragma once
#include "bvh_interface.h"
#include "config.h"
#include "draw.h"
#include "intersect.h"
#include "reservoir.h"
#include "scene.h"
#include "shading.h"

constexpr glm::vec3 CAMERA_RAY_HIT_COLOR        = {0.0f, 1.0f, 0.0f};
constexpr glm::vec3 CAMERA_RAY_NO_HIT_COLOR     = {1.0f, 0.0f, 0.0f};

// Light samplers
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color);
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color);

// ReSTIR per-pixel canonical samples
Reservoir genCanonicalSamples(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, bool intersectionOnly);
