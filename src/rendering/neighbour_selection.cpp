#include "neighbour_selection.h"

#include <random>


std::vector<glm::ivec2> indicesRandom(int32_t x, int32_t y,
                                         const glm::ivec2& windowResolution, const Features& features) {
    // Define the range of possible values based on window dimensions and resample radius
    std::random_device rd;
    std::mt19937 gen(rd());
    int32_t resampleRadiusCast = static_cast<int32_t>(features.spatialResampleRadius);
    std::uniform_int_distribution<> distrX(std::max(0, x - resampleRadiusCast),
                                           std::min(windowResolution.x - 1, x + resampleRadiusCast));
    std::uniform_int_distribution<> distrY(std::max(0, y - resampleRadiusCast),
                                           std::min(windowResolution.y - 1, y + resampleRadiusCast));

    // Create indices
    std::vector<glm::ivec2> indices;
    indices.reserve(features.numNeighboursToSample + 1);    // Assign enough space for current pixel AND neighbours
    indices.push_back(glm::ivec2(x, y));                    // Always include the pixel itself
    for (uint32_t candidateIdx = 0U; candidateIdx < features.numNeighboursToSample; candidateIdx++) {
        indices.push_back(glm::ivec2(distrX(gen), distrY(gen)));
    }
    return indices;
}

ResampleIndicesGrid generateResampleIndicesGrid(const glm::ivec2& windowResolution, const Features& features) {
    ResampleIndicesGrid resampleIndices(windowResolution.y, std::vector<std::vector<glm::ivec2>>(windowResolution.x));
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            resampleIndices[y][x] = indicesRandom(x, y, windowResolution, features);
        }
    }
    return resampleIndices;
}
