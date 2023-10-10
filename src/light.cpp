#include "config.h"
#include "light.h"
#include "reservoir.h"
#include "texture.h"
#include "utils.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <random>
#include <vector>


// samples a segment light source
// you should fill in the vectors position and color with the sampled position and color
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color) {
    float segFrac   = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    position        = glm::mix(segmentLight.endpoint0, segmentLight.endpoint1, segFrac);
    color           = glm::mix(segmentLight.color0, segmentLight.color1, segFrac);
}

// samples a parallelogram light source
// you should fill in the vectors position and color with the sampled position and color
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color) {
    float axOneFrac = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    float axTwoFrac = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    position            = parallelogramLight.v0 + (axOneFrac * parallelogramLight.edge01) + (axTwoFrac * parallelogramLight.edge02);
    glm::vec3 linLerp01 = glm::mix(parallelogramLight.color0, parallelogramLight.color1, axOneFrac);
    glm::vec3 linLerp23 = glm::mix(parallelogramLight.color2, parallelogramLight.color3, axOneFrac);
    color               = glm::mix(linLerp01, linLerp23, axTwoFrac);
}

// test the visibility at a given light sample
// returns true if sample is visible, false otherwise
bool testVisibilityLightSample(const glm::vec3& samplePos, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo) {
    // Construct shadow ray
    glm::vec3 shadingPoint  = ray.origin + (ray.t * ray.direction);
    glm::vec3 pointToSample = glm::normalize(samplePos - shadingPoint); 
    shadingPoint            += pointToSample * SHADOW_RAY_EPSILON;      // Small epsilon in shadow ray direction to avoid self-shadowing
    Ray shadowRay { shadingPoint, pointToSample, glm::distance(shadingPoint, samplePos) };

    // Visibility test and debug rays
    HitInfo shadowRayHit;
    bool visible = !bvh.intersect(shadowRay, shadowRayHit, features);
    if (visible) {
        drawRay(shadowRay, SHADOW_RAY_NO_HIT_COLOR);
        return true;
    }
    drawRay(shadowRay, SHADOW_RAY_INTERSECT_COLOR);
    return false;
}

glm::vec3 pointLightContribution(const PointLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo) {
    if (features.enableHardShadow && !testVisibilityLightSample(light.position, bvh, features, ray, hitInfo)) { return glm::vec3(0.0f); }
    return computeShading(light.position, light.color, features, ray, hitInfo);
}

glm::vec3 segmentLightContribution(const SegmentLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo) {
    // Generate sample
    glm::vec3 samplePos, sampleCol;
    sampleSegmentLight(light, samplePos, sampleCol);
    
    // Visibility and shading
    if (features.enableSoftShadow && !testVisibilityLightSample(samplePos, bvh, features, ray, hitInfo)) { return glm::vec3(0.0f); }
    return computeShading(samplePos, sampleCol, features, ray, hitInfo);
}

glm::vec3 parallelogramLightContribution(const ParallelogramLight& light, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo) {
    // Generate sample
    glm::vec3 samplePos, sampleCol;
    sampleParallelogramLight(light, samplePos, sampleCol);
    
    // Visibility and shading
    if (features.enableSoftShadow && !testVisibilityLightSample(samplePos, bvh, features, ray, hitInfo)) { return glm::vec3(0.0f); }
    return computeShading(samplePos, sampleCol, features, ray, hitInfo);
}

// Given an intersection, computes the contribution from all light sources at the intersection point
// in this method you should cycle the light sources and for each one compute their contribution
// don't forget to check for visibility (shadows!)
glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo) {
    if (!features.enableShading) { return hitInfo.material.kd; }

    // No lights to sample, just return black
    if (scene.lights.size() == 0UL) { return glm::vec3(0.0f); }
    
    // Uniform selection of light sources
    // TODO: Figure out better solution than uniform sampling (fuck you, point lights)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, scene.lights.size() - 1UL);

    // Properties of intersection point
    glm::vec3 intersectionPosition  = ray.origin + (ray.t * ray.direction);
    glm::vec3 diffuseColor          = features.enableTextureMapping && hitInfo.material.kdTexture                   ?
                                      acquireTexel(*hitInfo.material.kdTexture.get(), hitInfo.texCoord, features)   :
                                      hitInfo.material.kd;

    // Obtain initial light samples
    Reservoir reservoir;
    for (uint32_t sampleIdx = 0U; sampleIdx < features.initialLightSamples; sampleIdx++) {
        // Generate sample
        LightSample sample;
        const auto& light = scene.lights[distr(gen)];
        if (std::holds_alternative<PointLight>(light)) {
            const PointLight pointLight = std::get<PointLight>(light);
            sample.position             = pointLight.position;
            sample.color                = pointLight.color;
        } else if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            sampleSegmentLight(segmentLight, sample.position, sample.color);
        } else if (std::holds_alternative<ParallelogramLight>(light)) {
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
            sampleParallelogramLight(parallelogramLight, sample.position, sample.color);
        }

        // Update reservoir
        float sampleWeight = targetPDF(diffuseColor, sample, intersectionPosition) / (1.0f / static_cast<float>(scene.lights.size())); // We uniformly sample all lights, so distribution PDF is uniform
        reservoir.update(sample, sampleWeight); 
    }
    reservoir.outputWeight = (1.0f / targetPDF(diffuseColor, reservoir.outputSample, intersectionPosition)) * 
                             (1.0f / reservoir.numSamples) *
                             reservoir.wSum;

    // Visibility check for surviving canonical sample(s)
    if (features.initalSamplesVisibilityCheck && !testVisibilityLightSample(reservoir.outputSample.position, bvh, features, ray, hitInfo)) { reservoir.outputWeight = 0.0f; }

    // Final shading
    return computeShading(reservoir.outputSample.position, reservoir.outputSample.color, features, ray, hitInfo) * reservoir.outputWeight;
}
