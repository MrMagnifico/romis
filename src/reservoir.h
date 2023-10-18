#pragma once
#ifndef _RESERVOIR_H_
#define _RESERVOIR_H_

#include "common.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <framework/ray.h>

#include <limits>
#include <span>
#include <vector>

struct LightSample {
    glm::vec3 position  = {0.0f, 0.0f, 0.0f},
              color     = {0.0f, 0.0f, 0.0f};
};

struct Reservoir {
    // Intersection position info
    Ray cameraRay;
    HitInfo hitInfo;

    // Light sampling
    LightSample outputSample;
    float outputWeight  = 0.0f;
    float wSum          = std::numeric_limits<float>::min();    // Avoid division by zero issues
    size_t numSamples   = 1ULL;                                 // Avoid division by zero issues

    void update(LightSample sample, float weight);

    /**
     * Combine a number of reservoirs in a single final reservoir
     * 
     * @param reservoirs The reservoirs to be combined
     * @param finalReservoir Struct where final combined reservoir data will be stored. Should have the intersection position info of the relevant pixel
     * @param features Features configuration
    */
    static void combine(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const Features& features);
};

using ReservoirGrid = std::vector<std::vector<Reservoir>>;

float targetPDF(const LightSample& sample, const Ray& cameraRay, const HitInfo& hitInfo, const Features& features);

#endif
