#include "reservoir.h"
#include "shading.h"
#include "utils.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()
#include <cstdlib>
#include <iostream>


void Reservoir::update(LightSample sample, float weight) {
    numSamples          += 1ULL;
    wSum                += weight;
    float uniformRandom = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    if (uniformRandom < (weight / wSum)) { 
        outputSample.position   = sample.position;
        outputSample.color      = sample.color;
    }
}

void Reservoir::combineBiased(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const Features& features) {
    size_t totalSampleCount = 0ULL;
    for (const Reservoir& reservoir : reservoirStream) {
        float pdfValue      = targetPDF(reservoir.outputSample, finalReservoir.cameraRay, finalReservoir.hitInfo, features);
        totalSampleCount    += reservoir.numSamples;
        finalReservoir.update(reservoir.outputSample, pdfValue * reservoir.outputWeight * reservoir.numSamples);
    }

    float finalPdfValue         = targetPDF(finalReservoir.outputSample, finalReservoir.cameraRay, finalReservoir.hitInfo, features);
    finalPdfValue               = zeroWithinEpsilon(finalPdfValue) ? ZERO_EPSILON : finalPdfValue;
    finalReservoir.numSamples   = totalSampleCount;
    finalReservoir.outputWeight = (1.0f / finalPdfValue) * 
                                  (1.0f / finalReservoir.numSamples) *
                                  finalReservoir.wSum;
}

void Reservoir::combineUnbiased(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const BvhInterface& bvh, const Features& features) {
    size_t totalSampleCount = 0ULL;
    for (const Reservoir& reservoir : reservoirStream) {
        float pdfValue      = targetPDF(reservoir.outputSample, finalReservoir.cameraRay, finalReservoir.hitInfo, features);
        totalSampleCount    += reservoir.numSamples;
        finalReservoir.update(reservoir.outputSample, pdfValue * reservoir.outputWeight * reservoir.numSamples);
    }
    finalReservoir.numSamples = totalSampleCount;

    size_t contributingSampleCount = 1ULL; // Avoid division by zero
    for (const Reservoir& reservoir : reservoirStream) {
        if (features.spatialReuseVisibilityCheck && !testVisibilityLightSample(finalReservoir.outputSample.position, bvh, features, reservoir.cameraRay, reservoir.hitInfo)) { continue; }
        float finalSamplePdfAtPixel = targetPDF(finalReservoir.outputSample, reservoir.cameraRay, reservoir.hitInfo, features);
        if (!zeroWithinEpsilon(finalSamplePdfAtPixel)) { contributingSampleCount += reservoir.numSamples; }
    }

    float finalPdfValue                 = targetPDF(finalReservoir.outputSample, finalReservoir.cameraRay, finalReservoir.hitInfo, features);
    finalPdfValue                       = zeroWithinEpsilon(finalPdfValue) ? ZERO_EPSILON : finalPdfValue;
    finalReservoir.outputWeight         = (1.0f / finalPdfValue) * 
                                          (1.0f / contributingSampleCount) *
                                          finalReservoir.wSum;
}

float targetPDF(const LightSample& sample, const Ray& cameraRay, const HitInfo& hitInfo, const Features& features) {
    glm::vec3 bsdf = computeShading(sample.position, sample.color, features, cameraRay, hitInfo);
    return glm::length(bsdf);
}
