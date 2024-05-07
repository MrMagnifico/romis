#include "neighbour_selection.h"

#include <algorithm>
#include <random>


bool areSimilar(const RayHit& lhs, const RayHit& rhs, const Features& features) {
    // Same geometries
    if (features.neighbourSameGeometry && lhs.hit.geometryId != rhs.hit.geometryId) { return false; }
    
    // Depth difference
    float depthFracDiff = std::abs(1.0f - (lhs.ray.t / rhs.ray.t)); 
    if (depthFracDiff > features.neighbourMaxDepthDifferenceFraction) { return false; }

    // Normal angle difference
    float maxDiffCos        = std::cos(features.neighbourMaxNormalAngleDifferenceRadians);
    float normalsDotProd    = glm::dot(lhs.hit.normal, rhs.hit.normal);
    if (normalsDotProd < features.neighbourMaxNormalAngleDifferenceRadians) { return false; }

    // We passed the similarity test!
    return true;
}

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

std::vector<glm::ivec2> indicesSimilarity(int32_t x, int32_t y,
                                          const PrimaryHitGrid& primaryHits, const glm::ivec2& windowResolution, const Features& features) {
    // In extreme cases, all neighbours are similar or dissimilar. We reserve enough memory for either
    std::vector<glm::ivec2> similarIndices, dissimilarIndices;
    similarIndices.reserve(features.spatialResampleRadius * features.spatialResampleRadius * 4U);
    dissimilarIndices.reserve(features.spatialResampleRadius * features.spatialResampleRadius * 4U);

    // Ensure that our traversal does not exceed screen bounds
    int32_t resampleRadiusCast = static_cast<int32_t>(features.spatialResampleRadius);
    glm::ivec2 minExtents(std::max(0, x - resampleRadiusCast),                      std::max(0, y - resampleRadiusCast));
    glm::ivec2 maxExtents(std::min(windowResolution.x - 1, x + resampleRadiusCast), std::min(windowResolution.y - 1, y + resampleRadiusCast));
    
    // Categorise all pixels in neighbourhood
    const RayHit& canonical = primaryHits[y][x];
    for (int32_t neighbourY = minExtents.y; neighbourY <= maxExtents.y; neighbourY++) {
        for (int32_t neighbourX = minExtents.x; neighbourX <= maxExtents.x; neighbourX++) {
            // Skip canonical pixel
            if (neighbourY == y && neighbourX == x) { continue; }

            // Categorise neighbour
            glm::ivec2 neighbourCoords(neighbourX, neighbourY);
            const RayHit& neighbour = primaryHits[neighbourY][neighbourX]; 
            if (areSimilar(canonical, neighbour, features)) { similarIndices.push_back(neighbourCoords); }
            else                                            { dissimilarIndices.push_back(neighbourCoords); }
        }
    }

    // Create indices
    std::vector<glm::ivec2> indices;
    indices.reserve(features.numNeighboursToSample + 1);    // Assign enough space for current pixel AND neighbours
    indices.push_back(glm::ivec2(x, y));                    // Always include the pixel itself
    std::random_device rd;
    std::mt19937 rng(rd());
    switch (features.neighbourSelectionStrategy) {
        case NeighbourSelectionStrategy::Similar: {
            if (similarIndices.size() < features.numNeighboursToSample) {                                       // Not enough similar neighbours
                indices.insert(indices.end(), similarIndices.begin(), similarIndices.end());                    // Place however many we can
                std::sample(dissimilarIndices.begin(), dissimilarIndices.end(), std::back_inserter(indices),    // Make up for deficit from dissimilar neighbours
                            features.numNeighboursToSample - similarIndices.size(), rng); 
            } else { std::sample(similarIndices.begin(), similarIndices.end(), std::back_inserter(indices), features.numNeighboursToSample, rng); }
        } break;
        case NeighbourSelectionStrategy::Dissimilar: {
            if (dissimilarIndices.size() < features.numNeighboursToSample) {                            // Not enough dissimilar neighbours
                indices.insert(indices.end(), dissimilarIndices.begin(), dissimilarIndices.end());      // Place however many we can
                std::sample(similarIndices.begin(), similarIndices.end(), std::back_inserter(indices),  // Make up for deficit from similar neighbours
                            features.numNeighboursToSample - similarIndices.size(), rng);
            } else { std::sample(dissimilarIndices.begin(), dissimilarIndices.end(), std::back_inserter(indices), features.numNeighboursToSample, rng); }
        } break;
        case NeighbourSelectionStrategy::EqualSimilarDissimilar: {
            // Ensure there are sufficient quantities of similars and dissimilars to satisfy halfway split and make up for difference if that is not possible
            uint32_t similarsSampled    = std::min((features.numNeighboursToSample / 2U) + 1U, static_cast<uint32_t>(similarIndices.size()));
            uint32_t desiredDissimilars = features.numNeighboursToSample - similarsSampled;
            if (desiredDissimilars > dissimilarIndices.size()) { similarsSampled += features.numNeighboursToSample - dissimilarIndices.size() - similarsSampled; }

            std::sample(similarIndices.begin(),     similarIndices.end(),       std::back_inserter(indices), similarsSampled, rng);
            std::sample(dissimilarIndices.begin(),  dissimilarIndices.end(),    std::back_inserter(indices), features.numNeighboursToSample - similarsSampled, rng);
        } break;
        default: { throw std::runtime_error("indicesSimilarity called with unsupported neighbour selection strategy"); }
    }
    return indices;
}

ResampleIndicesGrid generateResampleIndicesGrid(const PrimaryHitGrid& primaryHits,
                                                const glm::ivec2& windowResolution, const Features& features) {
    ResampleIndicesGrid resampleIndices(windowResolution.y, std::vector<std::vector<glm::ivec2>>(windowResolution.x));
    bool useRandom = features.neighbourSelectionStrategy == NeighbourSelectionStrategy::Random;
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            resampleIndices[y][x] = useRandom                                       ?
                                    indicesRandom(x, y, windowResolution, features) :
                                    indicesSimilarity(x, y, primaryHits, windowResolution, features);
        }
    }
    return resampleIndices;
}
