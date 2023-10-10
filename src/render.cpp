#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif

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

ReservoirGrid renderRayTracing(const Scene& scene, const Trackball& camera,
                               const BvhInterface& bvh, const ReservoirGrid& previousFrameGrid, Screen& screen,
                               const Features& features) {
    ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);

    // Final shading
    glm::ivec2 windowResolution = screen.resolution();
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            const Reservoir& reservoir  = reservoirGrid[y][x];
            glm::vec3 finalColor        = computeShading(reservoir.outputSample.position, reservoir.outputSample.color, features, reservoir.cameraRay, reservoir.hitInfo) * reservoir.outputWeight;
            screen.setPixel(x, y, finalColor);
        }
    }

    // Return current frame's final grid for temporal reuse
    return reservoirGrid;
}