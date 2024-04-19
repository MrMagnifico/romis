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
    std::cout << "===== Rendering with ReSTIR =====" << std::endl;
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
    std::cout << std::endl;

    // Return current frame's final grid for temporal reuse
    return reservoirGrid;
}

void renderRMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));
    std::cout << "===== Rendering with R-MIS =====" << std::endl;

    for (uint32_t iteration = 0U; iteration < features.maxIterationsMIS; iteration++) {
        ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);

        std::cout << "Iteration " << iteration + 1 << std::endl;
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
        std::cout << std::endl;
    }
    combineToScreen(screen, finalPixelColors, features);
}

void renderROMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    // Used in both direct and progressive estimators
    constexpr int32_t TOTAL_NEIGHBOURS = 9; // TODO: Make this adjustable
    glm::ivec2 windowResolution = screen.resolution();
    MatrixGrid techniqueMatrices(windowResolution.y,        std::vector<Eigen::MatrixXf>(windowResolution.x, Eigen::MatrixXf::Zero(TOTAL_NEIGHBOURS, TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));

    // ===== PROGRESSIVE ONLY =====
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));
    VectorGrid alphaVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid alphaVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    VectorGrid alphaVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(TOTAL_NEIGHBOURS)));
    const int32_t totalSamples              = TOTAL_NEIGHBOURS * features.numSamplesInReservoir; // All pixels generate the same number of samples and sample the same number of neighbours
    const int32_t fractionOfTotalSamples    = features.numSamplesInReservoir / TOTAL_NEIGHBOURS; // Redundant? Yes. Helps with code clarity? Also yes

    std::cout << "===== Rendering with R-OMIS ====="   << std::endl;
    for (uint32_t iteration = 0U; iteration < features.maxIterationsMIS; iteration++) {
        ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);

        std::cout << "Iteration " << iteration + 1 << std::endl;
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

                // ===== PROGRESSIVE ONLY =====
                // Update alpha vector estimates
                if (features.useProgressiveROMIS && iteration >= 1U && iteration % features.progressiveUpdateMod == 0) {
                    alphaVectorsRed[y][x]   = solveSystem(techniqueMatrices[y][x], contributionVectorsRed[y][x]);
                    alphaVectorsGreen[y][x] = solveSystem(techniqueMatrices[y][x], contributionVectorsGreen[y][x]);
                    alphaVectorsBlue[y][x]  = solveSystem(techniqueMatrices[y][x], contributionVectorsBlue[y][x]);
                }
                
                // Construct elements of the technique matrix and contribution vector estimates
                for (size_t pixelIdx = 0ULL; pixelIdx < neighborhood.size(); pixelIdx++) {
                    // ===== PROGRESSIVE ONLY ===
                    // Add this pixel's portion of the sum of alpha to current iteration estimate
                    finalPixelColors[y][x] += glm::vec3(alphaVectorsRed[y][x](pixelIdx),
                                                        alphaVectorsGreen[y][x](pixelIdx),
                                                        alphaVectorsBlue[y][x](pixelIdx));

                    const Reservoir& pixel = neighborhood[pixelIdx];
                    for (size_t sampleIdx = 0ULL; sampleIdx < pixel.outputSamples.size(); sampleIdx++) {
                        const SampleData& sample = pixel.outputSamples[sampleIdx];

                        // Compute column vector of all sampling techniques evaluated with current sample
                        Eigen::VectorXf colVecW(neighborhood.size());
                        for (int32_t distributionIdx = 0ULL; distributionIdx < neighborhood.size(); distributionIdx++) {
                            const Reservoir& distribution   = neighborhood[distributionIdx];
                            colVecW(distributionIdx)        = arbitraryUnbiasedContributionWeightReciprocal(sample.lightSample, distribution, scene, sampleIdx, features);
                        }

                        // Evaluate shading (integrand function) for the current sample
                        glm::vec3 sampleColor = testVisibilityLightSample(sample.lightSample.position, bvh, features, primaryRay, primaryHitInfo)           ?
                                                computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, primaryHitInfo) :
                                                glm::vec3(0.0f);

                        // ===== PROGRESSIVE ONLY =====
                        // Add this sample's evaluation to current iteration estimate 
                        glm::vec3 sumAlphaProducts(0.0f);
                        float sumSampleFractionProducts = std::numeric_limits<float>::min();
                        for (int32_t distributionIdx = 0ULL; distributionIdx < neighborhood.size(); distributionIdx++) {
                            glm::vec3 alphaRGB(alphaVectorsRed[y][x](distributionIdx),
                                               alphaVectorsGreen[y][x](distributionIdx),
                                               alphaVectorsBlue[y][x](distributionIdx));
                            sumAlphaProducts            += alphaRGB * colVecW(distributionIdx);
                            sumSampleFractionProducts   += fractionOfTotalSamples * colVecW(distributionIdx);
                        }
                        finalPixelColors[y][x] += (1.0f / totalSamples) * // We divide by the total overall number of samples here because it makes the code a bit more readable. Slightly less efficient, but this REALLY isn't the sourcv of our computational woes
                                                  ((sampleColor / sumSampleFractionProducts) - (sumAlphaProducts / sumSampleFractionProducts));

                        // Compute scaling factor
                        float scaleFactor = std::numeric_limits<float>::min();
                        for (float techniqueEval : colVecW) { scaleFactor += features.numSamplesInReservoir * techniqueEval; } // Each reservoir/technique stores the same number of samples
                        scaleFactor = 1.0f / scaleFactor;

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
        std::cout << std::endl;
    }

    // Compute final results
    if (features.useProgressiveROMIS) { combineToScreen(screen, finalPixelColors, features); }
    else {
        // Solve linear system on per-pixel basis
        std::cout << "Integral component summation..." << std::endl;
        progressbar progressbarPixels(static_cast<int32_t>(windowResolution.y));
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Compute weighted contributions to final integral value for each color component 
                const Eigen::VectorXf integralComponentsRed     = solveSystem(techniqueMatrices[y][x], contributionVectorsRed[y][x]);
                const Eigen::VectorXf integralComponentsGreen   = solveSystem(techniqueMatrices[y][x], contributionVectorsGreen[y][x]);
                const Eigen::VectorXf integralComponentsBlue    = solveSystem(techniqueMatrices[y][x], contributionVectorsBlue[y][x]);

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
