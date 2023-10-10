#include "reservoir.h"
#include "utils.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()
#include <cstdlib>


void Reservoir::update(LightSample sample, float weight) {
    numSamples          += 1UL;
    wSum                += weight;
    float uniformRandom = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    if (uniformRandom < (weight / wSum)) { 
        outputSample.position   = sample.position;
        outputSample.color      = sample.color;
    }
}

void Reservoir::update(Reservoir inputReservoir, const Features& features) {
    // Compute target PDF of input reservoir's sample using current reservoir's intersection data
    glm::vec3 albedo                = diffuseAlbedo(hitInfo, features);
    glm::vec3 intersectionPosition  = cameraRay.origin + (cameraRay.t * cameraRay.direction);
    float pdfValue                  = targetPDF(albedo, inputReservoir.outputSample, intersectionPosition);

    // Update reservoir values
    update(inputReservoir.outputSample, pdfValue * inputReservoir.outputWeight * inputReservoir.numSamples);
    numSamples      += inputReservoir.numSamples;
    outputWeight    = (1.0f / targetPDF(albedo, outputSample, intersectionPosition)) * 
                      (1.0f / numSamples) *
                      wSum;
}

float targetPDF(glm::vec3 diffuseAlbedo, LightSample sample, glm::vec3 intersectionPosition) {
    float squaredDistance   = glm::distance(sample.position, intersectionPosition);
    squaredDistance         *= squaredDistance;
    return glm::dot(diffuseAlbedo, sample.color) * (1.0f / squaredDistance);
}
