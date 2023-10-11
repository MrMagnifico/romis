#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include <framework/trackball.h>

#ifdef NDEBUG
#include <omp.h>
#endif

#include <array>
#include <random>


ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    ReservoirGrid initialSamples(windowResolution.y, std::vector<Reservoir>(windowResolution.x));

    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            const glm::vec2 normalizedPixelPos { float(x) / float(windowResolution.x) * 2.0f - 1.0f,
                                                 float(y) / float(windowResolution.y) * 2.0f - 1.0f };
            const Ray cameraRay     = camera.generateRay(normalizedPixelPos);
            initialSamples[y][x]    = genCanonicalSamples(scene, bvh, features, cameraRay);
        }
    }
    return initialSamples;
}

void spatialReuse(ReservoirGrid& reservoirGrid, const BvhInterface& bvh, const Screen& screen, const Features& features) {
    // Uniform selection of neighbours in N pixel Manhattan distance radius
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(-features.spatialResampleRadius, features.spatialResampleRadius);

    glm::ivec2 windowResolution = screen.resolution();
    ReservoirGrid prevIteration = reservoirGrid;
    for (uint32_t pass = 0U; pass < features.spatialResamplingPasses; pass++) {
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Select candidates
                Reservoir& current = reservoirGrid[y][x];
                std::vector<Reservoir> selected(features.numNeighboursToSample + 1ULL);
                for (uint32_t neighbourCount = 0U; neighbourCount < features.numNeighboursToSample; neighbourCount++) {
                    int neighbourX              = std::clamp(x + distr(gen), 0, windowResolution.x - 1);
                    int neighbourY              = std::clamp(y + distr(gen), 0, windowResolution.y - 1);
                    Reservoir neighbour         = prevIteration[neighbourY][neighbourX]; // Create copy for local modification based on visiblity (if desired)
                    if (features.spatialReuseVisibilityCheck && !testVisibilityLightSample(neighbour.outputSample.position, bvh, features, current.cameraRay, current.hitInfo)) { neighbour.outputWeight = 0.0f; }
                    selected[neighbourCount]    = neighbour;
                }
                selected[features.numNeighboursToSample] = current; // Ensure pixel's own reservoir is also considered

                // Combine to single reservoir
                Reservoir combined;
                combined.cameraRay  = current.cameraRay;
                combined.hitInfo    = current.hitInfo;
                Reservoir::combine(selected, combined, features);
                reservoirGrid[y][x] = combined;
            }
        }
        prevIteration = reservoirGrid;
    }
}

void temporalReuse(ReservoirGrid& reservoirGrid, const ReservoirGrid& previousFrameGrid,
                   Screen& screen, const glm::vec2 motionVector, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // Compute previous corresponding pixels and skip temporal resampling if coordinates are out of bounds
            glm::ivec2 prevPixelCoords(x - static_cast<int>(motionVector.x), y - static_cast<int>(motionVector.y));
            if (prevPixelCoords.x < 0 || prevPixelCoords.x >= windowResolution.x ||
                prevPixelCoords.y < 0 || prevPixelCoords.y >= windowResolution.y) { continue; }

            // Locally clamp M value to 20x current frame's M to bound temporal creep
            Reservoir& current              = reservoirGrid[y][x];
            Reservoir temporalPredecessor   = previousFrameGrid[prevPixelCoords.y][prevPixelCoords.x];
            temporalPredecessor.numSamples  = std::clamp(temporalPredecessor.numSamples, 0ULL, 20ULL * current.numSamples);

            // Combine to single reservoir
            Reservoir combined;
            combined.cameraRay                              = current.cameraRay;
            combined.hitInfo                                = current.hitInfo;
            std::array<Reservoir, 2ULL> pixelAndPredecessor = { current, temporalPredecessor };
            Reservoir::combine(pixelAndPredecessor, combined, features);
            reservoirGrid[y][x]                             = combined;
        }
    }
}

ReservoirGrid renderRayTracing(std::shared_ptr<const ReservoirGrid> previousFrameGrid,
                               const Scene& scene, const Trackball& camera,
                               const BvhInterface& bvh, Screen& screen,
                               const glm::vec2 motionVector, const Features& features) {
    ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);
    if (features.temporalReuse && previousFrameGrid)    { temporalReuse(reservoirGrid, *previousFrameGrid.get(), screen, motionVector, features); }
    if (features.spatialReuse)                          { spatialReuse(reservoirGrid, bvh, screen, features); }

    // Final shading
    glm::ivec2 windowResolution = screen.resolution();
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            const Reservoir& reservoir  = reservoirGrid[y][x];
            glm::vec3 finalColor        = testVisibilityLightSample(reservoir.outputSample.position, bvh, features, reservoir.cameraRay, reservoir.hitInfo)                 ?
                                          computeShading(reservoir.outputSample.position, reservoir.outputSample.color, features, reservoir.cameraRay, reservoir.hitInfo)   :
                                          glm::vec3(0.0f);
            finalColor                  *= reservoir.outputWeight;
            screen.setPixel(x, y, finalColor);
        }
    }

    // Return current frame's final grid for temporal reuse
    return reservoirGrid;
}