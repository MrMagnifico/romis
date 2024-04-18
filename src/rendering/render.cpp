#include "render.h"

#ifdef NDEBUG
#include <omp.h>
#endif

#include <framework/trackball.h>

#include <post_processing/tone_mapping.h>
#include <rendering/render_utils.h>
#include <rendering/screen.h>
#include <scene/light.h>
#include <utils/magic_enum.hpp>
#include <utils/progressbar.hpp>
#include <utils/utils.h>

#include <array>
#include <iostream>


ReservoirGrid renderReSTIR(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                           const Scene& scene, const Trackball& camera,
                           const BvhInterface& bvh, Screen& screen,
                           const Features& features) {
    std::cout << "Rendering with ReSTIR..." << std::endl;
    ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);
    if (features.temporalReuse && previousFrameGrid)    { temporalReuse(reservoirGrid, *previousFrameGrid.get(), bvh, screen, features); }
    if (features.spatialReuse)                          { spatialReuse(reservoirGrid, bvh, screen, features); }

    // Final shading
    glm::ivec2 windowResolution = screen.resolution();
    std::cout << "Final shading..." << std::endl;
    progressbar progressBarPixels(windowResolution.y);
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // Compute shading from final sample(s)
            const Reservoir& reservoir  = reservoirGrid[y][x];
            glm::vec3 finalColor        = finalShading(reservoir, reservoir.cameraRay, bvh, features);

            // Apply tone mapping and set final pixel color
            if (features.enableToneMapping) { finalColor = exposureToneMapping(finalColor, features); }
            screen.setPixel(x, y, finalColor);
        }
        #pragma omp critical
        progressBarPixels.update();
    }

    // Return current frame's final grid for temporal reuse
    return reservoirGrid;
}

void renderRMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));
    std::cout << "Rendering with R-MIS..." << std::endl;

    for (uint32_t iteration = 0U; iteration < features.maxIterationsRMIS; iteration++) {
        ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);

        std::cout << std::endl
                  << "Iteration " << iteration + 1 << std::endl;
        progressbar progressBarPixels(windowResolution.y);
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Collect primary ray info
                const Ray& primaryRay           = reservoirGrid[y][x].cameraRay;
                const HitInfo& primaryHitInfo   = reservoirGrid[y][x].hitInfo;

                // Gather samples from all pixels in 3x3 neighborhood
                std::vector<Reservoir> neighborhood;
                neighborhood.reserve(9);
                for (int sampleY = y - 1; sampleY <= y + 1; sampleY++) {
                    for (int sampleX = x - 1; sampleX <= x + 1; sampleX++) {
                        int neighborY = std::clamp(sampleY, 0, windowResolution.y - 1);
                        int neighborX = std::clamp(sampleX, 0, windowResolution.x - 1);
                        neighborhood.push_back(reservoirGrid[neighborY][neighborX]);
                    }
                }

                // Combine shading results from all gathered pixels
                glm::vec3 finalColor(0.0f);
                for (const Reservoir& pixel : neighborhood) {
                    for (const SampleData& sample : pixel.outputSamples) {
                        // Compute MIS weight
                        float misWeight;
                        switch (features.misWeightRMIS) {
                            case MISWeightRMIS::Equal:      { misWeight = 1.0f / neighborhood.size(); } break;
                            case MISWeightRMIS::Balance:    { misWeight = generalisedBalanceHeuristic(sample.lightSample, neighborhood, primaryRay, primaryHitInfo, features); } break;
                            default:                        { throw std::runtime_error(std::format("Unhandled MIS weight type: {}", magic_enum::enum_name<MISWeightRMIS>(features.misWeightRMIS))); }
                        }

                        // Evaluate sample contribution
                        glm::vec3 sampleColor   = testVisibilityLightSample(sample.lightSample.position, bvh, features, primaryRay, primaryHitInfo)             ?
                                                  computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, primaryHitInfo)   :
                                                  glm::vec3(0.0f);
                        finalColor              += (misWeight * sampleColor * sample.outputWeight) / glm::vec3(static_cast<float>(pixel.outputSamples.size()));
                    }
                }

                // Accumulate iteration results
                finalPixelColors[y][x] += finalColor;

            }
            #pragma omp critical
            progressBarPixels.update();
        }
    }
    std::cout << std::endl;

    // Average iterations and write tone-mapped value to screen
    std::cout << "Iteration combination..." << std::endl;
    progressbar progressbarPixels(static_cast<int32_t>(windowResolution.y));
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            glm::vec3 finalColor = finalPixelColors[y][x] / static_cast<float>(features.maxIterationsRMIS);
            if (features.enableToneMapping) { finalColor = exposureToneMapping(finalColor, features); }
            screen.setPixel(x, y, finalColor);
        }
        #pragma omp critical
        progressbarPixels.update();
    }
}

void renderROMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    constexpr int32_t TOTAL_NEIGHBOURS = 9; // TODO: Make this adjustable
    glm::ivec2 windowResolution = screen.resolution();
    MatrixGrid techniqueMatrices(windowResolution.y,        std::vector<Eigen::MatrixXf>(windowResolution.x, Eigen::MatrixXf::Zero(TOTAL_NEIGHBOURS, TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));

    std::cout   << "Rendering with R-OMIS..."   << std::endl;
    for (uint32_t iteration = 0U; iteration < features.maxIterationsRMIS; iteration++) {
        ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);

        std::cout << std::endl
                  << "Iteration " << iteration + 1 << std::endl;
        progressbar progressbarPixels(static_cast<int32_t>(windowResolution.y));
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Collect primary ray info
                const Ray& primaryRay           = reservoirGrid[y][x].cameraRay;
                const HitInfo& primaryHitInfo   = reservoirGrid[y][x].hitInfo;

                // Gather samples from all pixels in 3x3 neighborhood
                std::vector<Reservoir> neighborhood;
                neighborhood.reserve(9);
                for (int sampleY = y - 1; sampleY <= y + 1; sampleY++) {
                    for (int sampleX = x - 1; sampleX <= x + 1; sampleX++) {
                        int neighborY = std::clamp(sampleY, 0, windowResolution.y - 1);
                        int neighborX = std::clamp(sampleX, 0, windowResolution.x - 1);
                        neighborhood.push_back(reservoirGrid[neighborY][neighborX]);
                    }
                }

                // Construct elements of the technique matrix and contribution vector estimates
                for (size_t pixelIdx = 0ULL; pixelIdx < neighborhood.size(); pixelIdx++) {
                    const Reservoir& pixel = neighborhood[pixelIdx];
                    for (size_t sampleIdx = 0ULL; sampleIdx < pixel.outputSamples.size(); sampleIdx++) {
                        const SampleData& sample = pixel.outputSamples[sampleIdx];

                        // Compute column vector of all sampling techniques evaluated with current sample
                        Eigen::VectorXf colVecW(neighborhood.size());
                        for (size_t techniqueIdx = 0ULL; techniqueIdx < neighborhood.size(); techniqueIdx++) {
                            const Reservoir& distribution   = neighborhood[techniqueIdx];
                            colVecW(techniqueIdx)           = arbitraryUnbiasedContributionWeightReciprocal(sample.lightSample, distribution, scene, sampleIdx, features);
                        }

                        // Compute scaling factor
                        float scaleFactor = std::numeric_limits<float>::min();
                        for (float techniqueEval : colVecW) { scaleFactor += features.numSamplesInReservoir * techniqueEval; } // Each reservoir/technique stores the same number of samples
                        scaleFactor = 1.0f / scaleFactor;

                        // Evaluate shading (integrand function) for the current sample
                        glm::vec3 sampleColor = testVisibilityLightSample(sample.lightSample.position, bvh, features, primaryRay, primaryHitInfo)           ?
                                                computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, primaryHitInfo) :
                                                glm::vec3(0.0f);

                        // Scale column vector and add to estimates
                        colVecW                 *= scaleFactor;
                        techniqueMatrices[y][x] += colVecW * colVecW.transpose();
                        for (int32_t rowIdx = 0; rowIdx < neighborhood.size(); rowIdx++) {
                            float scaleColVecConst                  = scaleFactor * colVecW(rowIdx);
                            contributionVectorsRed[y][x](rowIdx)    += sampleColor.r * scaleColVecConst;
                            contributionVectorsGreen[y][x](rowIdx)  += sampleColor.g * scaleColVecConst;
                            contributionVectorsBlue[y][x](rowIdx)   += sampleColor.b * scaleColVecConst;
                        }
                    }
                }
            }
            #pragma omp critical
            progressbarPixels.update();
        }
    }
    std::cout << std::endl;

    // Solve linear system on per-pixel basis
    std::cout << "Integral component summation..." << std::endl;
    progressbar progressbarPixels(static_cast<int32_t>(windowResolution.y));
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // Gather technique matrix and contribution vector for each color component 
            const Eigen::MatrixXf& tecMatrix        = techniqueMatrices[y][x];
            const Eigen::VectorXf& contribVecRed    = contributionVectorsRed[y][x];
            const Eigen::VectorXf& contribVecGreen  = contributionVectorsGreen[y][x];
            const Eigen::VectorXf& contribVecBlue   = contributionVectorsBlue[y][x];

            // Compute weighted contributions to final integral value for each color component 
            const Eigen::VectorXf integralComponentsRed     = tecMatrix.completeOrthogonalDecomposition().solve(contribVecRed);
            const Eigen::VectorXf integralComponentsGreen   = tecMatrix.completeOrthogonalDecomposition().solve(contribVecGreen);
            const Eigen::VectorXf integralComponentsBlue    = tecMatrix.completeOrthogonalDecomposition().solve(contribVecBlue);

            // Compute final color as sum of components
            glm::vec3 finalColor(0.0f);
            for (int32_t row = 0; row < TOTAL_NEIGHBOURS; row++) {
                finalColor.r += integralComponentsRed(row);
                finalColor.g += integralComponentsGreen(row);
                finalColor.b += integralComponentsBlue(row);
            }

            // Apply tone mapping and set final pixel color
            if (features.enableToneMapping) { finalColor = exposureToneMapping(finalColor, features); }
            screen.setPixel(x, y, finalColor);
        }
        #pragma omp critical
        progressbarPixels.update();
    }
    std::cout << std::endl;
}

std::optional<ReservoirGrid> renderRayTraced(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                                             const Scene& scene, const Trackball& camera,
                                             const BvhInterface& bvh, Screen& screen,
                                             const Features& features) {
    switch (features.rayTraceMode) {
        case RayTraceMode::ReSTIR: {
            return renderReSTIR(previousFrameGrid, scene, camera, bvh, screen, features);
        } break;
        case RayTraceMode::RMIS: {
            renderRMIS(scene, camera, bvh, screen, features);
            return std::nullopt;
        } break;
        case RayTraceMode::ROMIS: {
            renderROMIS(scene, camera, bvh, screen, features);
            return std::nullopt;
        }
        default: {
            throw std::runtime_error("Unsupported ray-tracing render mode requested from entry point");
        }
    }
}
