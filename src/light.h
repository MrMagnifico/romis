#pragma once
#include "bvh_interface.h"
#include "config.h"
#include "draw.h"
#include "intersect.h"
#include "scene.h"
#include "shading.h"

constexpr float SHADOW_RAY_EPSILON              = 1E-3F;
constexpr glm::vec3 SHADOW_RAY_INTERSECT_COLOR  = {1.0f, 0.0f, 0.0f};
constexpr glm::vec3 SHADOW_RAY_NO_HIT_COLOR     = {0.0f, 1.0f, 1.0f};

void sampleSegmentLight (const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color);
void sampleParallelogramLight (const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color);

bool testVisibilityLightSample(const glm::vec3& samplePos, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo);

glm::vec3 pointLightContribution(const PointLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo);
glm::vec3 segmentLightContribution(const SegmentLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo);
glm::vec3 parallelogramLightContribution(const ParallelogramLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo);


glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo);

