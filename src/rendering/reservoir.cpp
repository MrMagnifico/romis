#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()

#include <rendering/reservoir.h>
#include <rendering/shading.h>
#include <utils/utils.h>

size_t Reservoir::update(LightSample sample, float weight) {
    // Find reservoir with smallest weight sum
    size_t smallestWeightIdx    = 0ULL;
    float smallestWeight        = std::numeric_limits<float>::max();
    for (size_t reservoirIdx = 0ULL; reservoirIdx < wSums.size(); reservoirIdx++) {
        if (wSums[reservoirIdx] < smallestWeight) {
            smallestWeightIdx   = reservoirIdx;
            smallestWeight      = wSums[reservoirIdx];
        }
    }

    // Selected reservoir processes sample
    sampleNums[smallestWeightIdx]   += 1ULL;
    wSums[smallestWeightIdx]        += weight;
    float uniformRandom             = linearMap(static_cast<float>(rand()), 0.0f, RAND_MAX, 0.0f, 1.0f);
    if (uniformRandom < (weight / wSums[smallestWeightIdx] )) { 
        outputSamples[smallestWeightIdx].lightSample.position   = sample.position;
        outputSamples[smallestWeightIdx].lightSample.color      = sample.color;
    }

    return smallestWeightIdx;
}

size_t Reservoir::totalSampleNums() const {
    size_t sampleCountSum = 0ULL;
    for (size_t sampleNum : sampleNums) { sampleCountSum += sampleNum; }
    return sampleCountSum;
}

void Reservoir::combineBiased(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const Features& features) {
    // Process reservoir stream sample-by-sample
    std::vector<size_t> totalSampleCounts(finalReservoir.outputSamples.size(), 0ULL);
    for (const Reservoir& reservoir : reservoirStream) {
        for (size_t sampleIdx = 0ULL; sampleIdx < reservoir.outputSamples.size(); sampleIdx++) {
            float pdfValue = targetPDF(reservoir.outputSamples[sampleIdx].lightSample,
                                       finalReservoir.cameraRay, finalReservoir.hitInfo,
                                       features);
            size_t updatedFinalReservoirIdx = finalReservoir.update(
                reservoir.outputSamples[sampleIdx].lightSample,
                pdfValue * reservoir.outputSamples[sampleIdx].outputWeight * reservoir.sampleNums[sampleIdx]);
            totalSampleCounts[updatedFinalReservoirIdx] += reservoir.sampleNums[sampleIdx];
        }
    }
    finalReservoir.sampleNums = std::move(totalSampleCounts);

    // Compute unbiased constribution weights for each sample in the final reservoir
    for (size_t reservoirIdx = 0ULL; reservoirIdx < finalReservoir.outputSamples.size(); reservoirIdx++) {
        float finalPdfValue = targetPDF(finalReservoir.outputSamples[reservoirIdx].lightSample,
                                        finalReservoir.cameraRay, finalReservoir.hitInfo,
                                        features);
        if (finalPdfValue == 0.0f)  { finalReservoir.outputSamples[reservoirIdx].outputWeight = 0.0f; }
        else                        { finalReservoir.outputSamples[reservoirIdx].outputWeight = (1.0f / finalPdfValue) * 
                                                                                                (1.0f / finalReservoir.sampleNums[reservoirIdx]) *
                                                                                                finalReservoir.wSums[reservoirIdx]; }
    }
}

void Reservoir::combineUnbiased(const std::span<Reservoir>& reservoirStream, Reservoir& finalReservoir, const BvhInterface& bvh, const Features& features) {
    // Process reservoir stream sample-by-sample
    std::vector<size_t> totalSampleCounts(finalReservoir.outputSamples.size(), 0ULL);
    for (const Reservoir& reservoir : reservoirStream) {
        for (size_t sampleIdx = 0ULL; sampleIdx < reservoir.outputSamples.size(); sampleIdx++) {
            float pdfValue = targetPDF(reservoir.outputSamples[sampleIdx].lightSample,
                                       finalReservoir.cameraRay, finalReservoir.hitInfo,
                                       features);
            size_t updatedFinalReservoirIdx = finalReservoir.update(
                reservoir.outputSamples[sampleIdx].lightSample,
                pdfValue * reservoir.outputSamples[sampleIdx].outputWeight * reservoir.sampleNums[sampleIdx]);
            totalSampleCounts[updatedFinalReservoirIdx] += reservoir.sampleNums[sampleIdx];
        }
    }
    finalReservoir.sampleNums = std::move(totalSampleCounts);

    // Count only samples which have a contribution in the final reservoir's domain for use in the unbiased contribution weights
    std::vector<size_t> numValidSamples(finalReservoir.outputSamples.size(), 0ULL);
    for (const Reservoir& reservoir : reservoirStream) {
        for (size_t outputSampleIdx = 0ULL; outputSampleIdx < finalReservoir.outputSamples.size(); outputSampleIdx++) {
            const SampleData& finalReservoirSample  = finalReservoir.outputSamples[outputSampleIdx]; 
            float pdfValue                          = targetPDF(finalReservoirSample.lightSample, reservoir.cameraRay, reservoir.hitInfo, features);
            if (features.spatialReuseVisibilityCheck)   { pdfValue *= testVisibilityLightSample(finalReservoirSample.lightSample.position, bvh, features, reservoir.cameraRay, reservoir.hitInfo); }
            if (pdfValue > 0.0f)                        { numValidSamples[outputSampleIdx] += reservoir.totalSampleNums(); }
        }
    }
    
    // Compute unbiased constribution weights for each sample in the final reservoir
    for (size_t outputSampleIdx = 0ULL; outputSampleIdx < finalReservoir.outputSamples.size(); outputSampleIdx++) {
        SampleData& finalReservoirSample    = finalReservoir.outputSamples[outputSampleIdx];
        float finalPdfValue                 = targetPDF(finalReservoirSample.lightSample, finalReservoir.cameraRay, finalReservoir.hitInfo, features);
        if (finalPdfValue == 0.0f || numValidSamples[outputSampleIdx] == 0ULL)  { finalReservoirSample.outputWeight = 0.0f; }
        else                                                                    { finalReservoirSample.outputWeight = (1.0f / finalPdfValue) * 
                                                                                                                      (1.0f / numValidSamples[outputSampleIdx]) *
                                                                                                                      finalReservoir.wSums[outputSampleIdx]; }
    }
}

float targetPDF(const LightSample& sample, const Ray& cameraRay, const HitInfo& hitInfo, const Features& features) {
    glm::vec3 bsdf = computeShading(sample.position, sample.color, features, cameraRay, hitInfo);
    return glm::length(bsdf);
}
