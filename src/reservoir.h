#pragma once
#ifndef _RESERVOIR_H_
#define _RESERVOIR_H_

#include "common.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <framework/ray.h>
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
    float wSum          = 0.0f;
    size_t numSamples   = 0UL;

    void update(Reservoir reservoir);
    void update(LightSample sample, float weight);
};

using ReservoirGrid = std::vector<std::vector<Reservoir>>;

float targetPDF(glm::vec3 diffuseAlbedo, LightSample sample, glm::vec3 intersectionPosition);

#endif
