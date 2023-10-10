#include "reservoir.h"
#include "utils.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()
#include <cstdlib>


void Reservoir::update(LightSample sample, float weight) {
    numSamples          += 1ULL;
    wSum                += weight;
    float uniformRandom = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    if (uniformRandom < (weight / wSum)) { 
        outputSample.position   = sample.position;
        outputSample.color      = sample.color;
    }
}

void Reservoir::combine(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const Features& features) {
    glm::vec3 albedo                = diffuseAlbedo(finalReservoir.hitInfo, features);
    glm::vec3 intersectionPosition  = finalReservoir.cameraRay.origin + (finalReservoir.cameraRay.t * finalReservoir.cameraRay.direction);
    size_t totalSampleCount         = 0ULL;
    for (const Reservoir& reservoir : reservoirStream) {
        float pdfValue      = targetPDF(albedo, reservoir.outputSample, intersectionPosition);
        totalSampleCount    += reservoir.numSamples;
        finalReservoir.update(reservoir.outputSample, pdfValue * reservoir.outputWeight * reservoir.numSamples);
    }
    finalReservoir.numSamples   = totalSampleCount;
    finalReservoir.outputWeight = (1.0f / targetPDF(albedo, finalReservoir.outputSample, intersectionPosition)) * 
                                  (1.0f / finalReservoir.numSamples) *
                                  finalReservoir.wSum;
}

float targetPDF(glm::vec3 diffuseAlbedo, LightSample sample, glm::vec3 intersectionPosition) {
    float squaredDistance   = glm::distance(sample.position, intersectionPosition);
    squaredDistance         *= squaredDistance;
    return glm::dot(diffuseAlbedo, sample.color) * (1.0f / squaredDistance);
}
