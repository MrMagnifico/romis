#include "render.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <cereal/archives/json.hpp>
DISABLE_WARNINGS_POP()

#ifdef NDEBUG
#include <omp.h>
#endif

#include <framework/trackball.h>

#include <post_processing/tone_mapping.h>
#include <rendering/neighbour_selection.h>
#include <rendering/render_utils.h>
#include <rendering/screen.h>
#include <scene/light.h>
#include <utils/magic_enum.hpp>
#include <utils/progressbar.hpp>
#include <utils/utils.h>

#include <array>
#include <fstream>
#include <iostream>


ReservoirGrid renderReSTIR(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                           const Scene& scene, const Trackball& camera,
                           const EmbreeInterface& embreeInterface, Screen& screen,
                           const Features& features) {
    std::cout << "===== Rendering with ReSTIR =====" << std::endl;
    glm::ivec2 windowResolution         = screen.resolution();
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));
    ReservoirGrid currentGrid;

    auto start      = std::chrono::high_resolution_clock::now();
    PrimaryHitGrid primaryHits  = genPrimaryRayHits(scene, camera, embreeInterface, screen, features);
    auto end        = std::chrono::high_resolution_clock::now();
    auto duration   = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Primary rays time: {}ms\n", duration);

    for (uint32_t iteration = 0U; iteration < features.maxIterations; iteration++) {
        std::cout << "= Iteration " << iteration + 1 << std::endl;

        // Carry out ReSTIR steps
        // Initial candidate generation
        start = std::chrono::high_resolution_clock::now();
        currentGrid = genInitialSamples(primaryHits, scene, embreeInterface, features, screen.resolution());
        end      = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Initial samples time: {}ms\n", duration);
        // Temporal resampling
        start = std::chrono::high_resolution_clock::now();
        if (features.temporalReuse && previousFrameGrid)    { temporalReuse(currentGrid, *previousFrameGrid.get(), embreeInterface, screen, features); }
        end      = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Temporal reuse time: {}ms\n", duration);
        // Spatial resampling
        start = std::chrono::high_resolution_clock::now();
        if (features.spatialReuse)                          { spatialReuse(currentGrid, embreeInterface, screen, features); }
        end      = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Spatial reuse time: {}ms\n", duration);

        std::cout << "Shading final samples..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Accumulate shading from final sample(s) to running sum
                const Reservoir& reservoir  = currentGrid[y][x];
                finalPixelColors[y][x]      += finalShadingReSTIR(reservoir, reservoir.cameraRay, embreeInterface, features);
            }
        }
        end      = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Final shading time: {}ms\n", duration);
    }

    // Produce final color and return current frame's final grid for temporal reuse
    start   = std::chrono::high_resolution_clock::now();
    combineToScreen(screen, finalPixelColors, features);
    end      = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Iteration combination time: {}ms\n", duration);
    return currentGrid;
}

void renderRMIS(const Scene& scene, const Trackball& camera, const EmbreeInterface& embreeInterface, Screen& screen, const Features& features) {
    std::cout << "===== Rendering with R-MIS =====" << std::endl;
    glm::ivec2 windowResolution         = screen.resolution();
    const uint32_t totalDistributions   = features.numNeighboursToSample + 1U; // Original pixel and neighbours
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));

    // Generate primary hits
    auto start      = std::chrono::high_resolution_clock::now();
    PrimaryHitGrid primaryHits          = genPrimaryRayHits(scene, camera, embreeInterface, screen, features);
    auto end        = std::chrono::high_resolution_clock::now();
    auto duration   = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Primary rays time: {}ms\n", duration);

    // Generate indices of neighbours to resample from
    start       = std::chrono::high_resolution_clock::now();
    ResampleIndicesGrid resampleIndices = generateResampleIndicesGrid(primaryHits, windowResolution, features);
    end         = std::chrono::high_resolution_clock::now();
    duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Resample indices grid time: {}ms\n", duration);


    for (uint32_t iteration = 0U; iteration < features.maxIterations; iteration++) {
        std::cout << "= Iteration " << iteration + 1 << std::endl;

        start       = std::chrono::high_resolution_clock::now();
        ReservoirGrid reservoirGrid = genInitialSamples(primaryHits, scene, embreeInterface, features, windowResolution);
        end         = std::chrono::high_resolution_clock::now();
        duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Initial samples time: {}ms\n", duration);

        start = std::chrono::high_resolution_clock::now();
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Collect primary ray info
                const Ray& primaryRay           = reservoirGrid[y][x].cameraRay;
                const HitInfo& primaryHitInfo   = reservoirGrid[y][x].hitInfo;

                // Gather samples from neighborhood defined by resample radius
                std::vector<Reservoir> neighborhood;
                neighborhood.reserve(totalDistributions); // Include space for the current pixel AND neighbours
                for (const glm::ivec2& index : resampleIndices[y][x]) { neighborhood.push_back(reservoirGrid[index.y][index.x]); }

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
                        glm::vec3 sampleColor   = testVisibilityLightSample(sample.lightSample.position, embreeInterface, features, primaryRay, primaryHitInfo)             ?
                                                  computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, primaryHitInfo)   :
                                                  glm::vec3(0.0f);
                        finalColor              += (misWeight * sampleColor * sample.outputWeight) / glm::vec3(static_cast<float>(pixel.outputSamples.size()));
                    }
                }

                // Accumulate iteration results
                finalPixelColors[y][x] += finalColor;
            }
        }

        // Time iteration
        end         = std::chrono::high_resolution_clock::now();
        duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# RMIS shading time: {}ms\n", duration);
    }

    // Combine final results to screen
    start = std::chrono::high_resolution_clock::now();
    combineToScreen(screen, finalPixelColors, features);
    end      = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Iteration combination time: {}ms\n", duration);
}

void renderROMIS(const Scene& scene, const Trackball& camera, const EmbreeInterface& embreeInterface, Screen& screen, const Features& features) {
    // ===== Used in both direct and progressive estimators =====
    std::cout << "===== Rendering with R-OMIS ====="   << std::endl;
    glm::ivec2 windowResolution             = screen.resolution();
    const uint32_t totalDistributions       = features.numNeighboursToSample + 1U; // Original pixel and neighbours
    MatrixGrid techniqueMatrices(windowResolution.y,        std::vector<Eigen::MatrixXf>(windowResolution.x, Eigen::MatrixXf::Zero(totalDistributions, totalDistributions)));
    VectorGrid contributionVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    VectorGrid contributionVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    VectorGrid contributionVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    // Generate primary hits
    auto start      = std::chrono::high_resolution_clock::now();
    PrimaryHitGrid primaryHits          = genPrimaryRayHits(scene, camera, embreeInterface, screen, features);
    auto end        = std::chrono::high_resolution_clock::now();
    auto duration   = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Primary rays time: {}ms\n", duration);
    // Generate indices of neighbours to resample from
    start       = std::chrono::high_resolution_clock::now();
    ResampleIndicesGrid resampleIndices = generateResampleIndicesGrid(primaryHits, windowResolution, features);
    end         = std::chrono::high_resolution_clock::now();
    duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Resample indices grid time: {}ms\n", duration);

    // ===== PROGRESSIVE ONLY =====
    PixelGrid finalPixelColors(windowResolution.y,   std::vector<glm::vec3>(windowResolution.x, glm::vec3(0.0f)));
    VectorGrid alphaVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    VectorGrid alphaVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    VectorGrid alphaVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf::Zero(totalDistributions)));
    const int32_t totalSamples              = totalDistributions * features.numSamplesInReservoir; // All pixels generate the same number of samples and sample the same number of neighbours
    const int32_t fractionOfTotalSamples    = features.numSamplesInReservoir / totalDistributions; // Redundant? Yes. Helps with code clarity? Also yes

    for (uint32_t iteration = 0U; iteration < features.maxIterations; iteration++) {
        std::cout << "= Iteration " << iteration + 1 << std::endl;

        start       = std::chrono::high_resolution_clock::now();
        ReservoirGrid reservoirGrid = genInitialSamples(primaryHits, scene, embreeInterface, features, windowResolution);
        end         = std::chrono::high_resolution_clock::now();
        duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# Initial samples time: {}ms\n", duration);

        start = std::chrono::high_resolution_clock::now();
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Collect primary ray info
                const Ray& primaryRay           = reservoirGrid[y][x].cameraRay;
                const HitInfo& primaryHitInfo   = reservoirGrid[y][x].hitInfo;

                // Gather samples from neighborhood defined by resample radius
                std::vector<Reservoir> neighborhood;
                neighborhood.reserve(totalDistributions);
                for (const glm::ivec2& index : resampleIndices[y][x]) { neighborhood.push_back(reservoirGrid[index.y][index.x]); }

                // ===== PROGRESSIVE ONLY =====
                // Update alpha vector estimates
                if (features.useProgressiveROMIS && iteration >= 1U && iteration % features.progressiveUpdateMod == 0) {
                    alphaVectorsRed[y][x]   = solveSystem(techniqueMatrices[y][x], contributionVectorsRed[y][x]);
                    alphaVectorsGreen[y][x] = solveSystem(techniqueMatrices[y][x], contributionVectorsGreen[y][x]);
                    alphaVectorsBlue[y][x]  = solveSystem(techniqueMatrices[y][x], contributionVectorsBlue[y][x]);
                }
                
                // Construct elements of the technique matrix and contribution vector estimates
                for (size_t pixelIdx = 0ULL; pixelIdx < totalDistributions; pixelIdx++) {
                    // ===== PROGRESSIVE ONLY ===
                    // Add this pixel's portion of the sum of alpha to current iteration estimate
                    finalPixelColors[y][x] += glm::vec3(alphaVectorsRed[y][x](pixelIdx),
                                                        alphaVectorsGreen[y][x](pixelIdx),
                                                        alphaVectorsBlue[y][x](pixelIdx));

                    const Reservoir& pixel = neighborhood[pixelIdx];
                    for (size_t sampleIdx = 0ULL; sampleIdx < pixel.outputSamples.size(); sampleIdx++) {
                        const SampleData& sample = pixel.outputSamples[sampleIdx];

                        // Compute column vector of all sampling techniques evaluated with current sample
                        Eigen::VectorXf colVecW(totalDistributions);
                        for (int32_t distributionIdx = 0ULL; distributionIdx < totalDistributions; distributionIdx++) {
                            const Reservoir& distribution   = neighborhood[distributionIdx];
                            colVecW(distributionIdx)        = arbitraryUnbiasedContributionWeightReciprocal(sample.lightSample, distribution, scene, sampleIdx, features);
                        }

                        // Evaluate shading (integrand function) for the current sample
                        glm::vec3 sampleColor = testVisibilityLightSample(sample.lightSample.position, embreeInterface, features, primaryRay, primaryHitInfo)           ?
                                                computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, primaryHitInfo) :
                                                glm::vec3(0.0f);

                        // ===== PROGRESSIVE ONLY =====
                        // Add this sample's evaluation to current iteration estimate 
                        glm::vec3 sumAlphaProducts(0.0f);
                        float sumSampleFractionProducts = std::numeric_limits<float>::min();
                        for (int32_t distributionIdx = 0ULL; distributionIdx < totalDistributions; distributionIdx++) {
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
                        for (int32_t rowIdx = 0; rowIdx < totalDistributions; rowIdx++) {
                            float scaleColVecConst                  = scaleFactor * colVecW(rowIdx);
                            contributionVectorsRed[y][x](rowIdx)    += sampleColor.r * scaleColVecConst;
                            contributionVectorsGreen[y][x](rowIdx)  += sampleColor.g * scaleColVecConst;
                            contributionVectorsBlue[y][x](rowIdx)   += sampleColor.b * scaleColVecConst;
                        }
                    }
                }
            }
        }
        // Save alphas visualisation if requested
        if (features.saveAlphasVisualisation) { visualiseAlphas(techniqueMatrices, contributionVectorsRed, contributionVectorsGreen, contributionVectorsBlue,
                                                                windowResolution, features); }

        // Time iteration
        end         = std::chrono::high_resolution_clock::now();
        duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::format("# ROMIS shading time: {}ms\n", duration);
    }

    // Compute final results
    start = std::chrono::high_resolution_clock::now();
    if (features.useProgressiveROMIS) { combineToScreen(screen, finalPixelColors, features); }
    else {
        // Solve linear system on per-pixel basis
        std::cout << "Integral component summation..." << std::endl;
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
                for (int32_t row = 0; row < totalDistributions; row++) {
                    finalColor.r += integralComponentsRed(row);
                    finalColor.g += integralComponentsGreen(row);
                    finalColor.b += integralComponentsBlue(row);
                }

                // Apply tone mapping and set final pixel color
                if (features.enableToneMapping) { finalColor = exposureToneMapping(finalColor, features); }
                screen.setPixel(x, y, finalColor);
            }
        }
    }
    // Time iteration combination
    end         = std::chrono::high_resolution_clock::now();
    duration    = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << std::format("# Iteration combination time: {}ms\n", duration);
}


std::optional<ReservoirGrid> renderRayTraced(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                                             const Scene& scene, const Trackball& camera,
                                             const EmbreeInterface& embreeInterface, Screen& screen,
                                             const Features& features) {
    // Render with desired mode
    std::optional<ReservoirGrid> finalReservoirs = std::nullopt;
    switch (features.rayTraceMode) {
        case RayTraceMode::ReSTIR:  { finalReservoirs = renderReSTIR(previousFrameGrid, scene, camera, embreeInterface, screen, features); } break;
        case RayTraceMode::RMIS:    { renderRMIS(scene, camera, embreeInterface, screen, features); } break;
        case RayTraceMode::ROMIS:   { renderROMIS(scene, camera, embreeInterface, screen, features); } break;
        default:                    { throw std::runtime_error("Unsupported ray-tracing render mode requested from entry point"); }
    }

    // Save used configuration to timestamped file
    std::filesystem::path renderDirPath = RENDERS_DIR;
    if (!std::filesystem::exists(renderDirPath)) { std::filesystem::create_directory(renderDirPath); }
    std::filesystem::path configFilePath = renderDirPath / (currentTime() + ".json");
    std::ofstream fileStream(configFilePath, std::ios::binary);
    cereal::JSONOutputArchive configArchive(fileStream);
    features.serialize(configArchive);

    return finalReservoirs;
}
