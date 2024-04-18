
#include <framework/trackball.h>

#ifdef NDEBUG
#include <omp.h>
#endif

#include <post_processing/tone_mapping.h>
#include <ray_tracing/intersect.h>
#include <rendering/render.h>
#include <rendering/screen.h>
#include <scene/light.h>
#include <utils/magic_enum.hpp>
#include <utils/progressbar.hpp>
#include <utils/utils.h>

#include <array>
#include <iostream>
#include <random>


ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    ReservoirGrid initialSamples(windowResolution.y, std::vector<Reservoir>(windowResolution.x, Reservoir(features.numSamplesInReservoir)));

    progressbar progressbar(windowResolution.y);
    std::cout << "Initial sample generation..." << std::endl;
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
        #pragma omp critical
        progressbar.update();
    }
    return initialSamples;
}

glm::vec3 finalShading(const Reservoir& reservoir, const Ray& primaryRay, const BvhInterface& bvh, const Features& features) {
    glm::vec3 finalColor(0.0f);
    for (const SampleData& sample : reservoir.outputSamples) {
        glm::vec3 sampleColor   = testVisibilityLightSample(sample.lightSample.position, bvh, features, primaryRay, reservoir.hitInfo)              ?
                                  computeShading(sample.lightSample.position, sample.lightSample.color, features, primaryRay, reservoir.hitInfo)    :
                                  glm::vec3(0.0f);
        sampleColor             *= sample.outputWeight;
        finalColor              += sampleColor;
    }
    finalColor /= reservoir.outputSamples.size(); // Divide final shading value by number of samples
    return finalColor;
}

void spatialReuse(ReservoirGrid& reservoirGrid, const BvhInterface& bvh, const Screen& screen, const Features& features) {
    // Uniform selection of neighbours in N pixel Manhattan distance radius
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(-static_cast<int32_t>(features.spatialResampleRadius), features.spatialResampleRadius);

    glm::ivec2 windowResolution = screen.resolution();
    ReservoirGrid prevIteration = reservoirGrid;
    for (uint32_t pass = 0U; pass < features.spatialResamplingPasses; pass++) {
        #ifdef NDEBUG
        #pragma omp parallel for schedule(guided)
        #endif
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // Select candidates
                std::vector<Reservoir> selected;
                selected.reserve(features.numNeighboursToSample + 1U); // Reserve memory needed for maximum possible number of samples (neighbours + current)
                Reservoir& current = reservoirGrid[y][x];
                for (uint32_t neighbourCount = 0U; neighbourCount < features.numNeighboursToSample; neighbourCount++) {
                    int neighbourX              = std::clamp(x + distr(gen), 0, windowResolution.x - 1);
                    int neighbourY              = std::clamp(y + distr(gen), 0, windowResolution.y - 1);
                    Reservoir neighbour         = prevIteration[neighbourY][neighbourX]; // Create copy for local modification
                    
                    // Conduct heuristic check if biased combination is used
                    if (!features.unbiasedCombination) { 
                        float depthFracDiff     = std::abs(1.0f - (neighbour.cameraRay.t / current.cameraRay.t));   // Check depth difference (greater than 10% leads to rejection) 
                        float normalsDotProd    = glm::dot(neighbour.hitInfo.normal, current.hitInfo.normal);       // Check normal difference (greater than 25 degrees leads to rejection)
                        if (depthFracDiff > 0.1f || normalsDotProd < 0.90630778703f) { continue; } 
                    }
                    
                    selected.push_back(neighbour);
                }

                // Ensure pixel's own reservoir is also considered
                selected.push_back(current);

                // Combine to single reservoir (biased or unbiased depending on user selection)
                Reservoir combined(current.outputSamples.size());
                combined.cameraRay  = current.cameraRay;
                combined.hitInfo    = current.hitInfo;
                if (features.unbiasedCombination)   { Reservoir::combineUnbiased(selected, combined, bvh, features); }
                else                                { Reservoir::combineBiased(selected, combined, features); }
                reservoirGrid[y][x] = combined;
            }
        }
        prevIteration = reservoirGrid;
    }
}

void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid, const BvhInterface& bvh,
                   Screen& screen, const Features& features) {
    glm::ivec2 windowResolution = screen.resolution();
    #ifdef NDEBUG
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // Clamp M and wSum values to a user-defined multiple of the current frame's to bound temporal creep
            Reservoir& current              = reservoirGrid[y][x];
            Reservoir& temporalPredecessor  = previousFrameGrid[y][x];
            size_t multipleCurrentM         = (features.temporalClampM * current.totalSampleNums()) + 1ULL;
            if (temporalPredecessor.totalSampleNums() > multipleCurrentM) {
                for (size_t reservoirIdx = 0ULL; reservoirIdx < temporalPredecessor.outputSamples.size(); reservoirIdx++) {
                    if (temporalPredecessor.sampleNums[reservoirIdx] == 0ULL) { continue; } // Samples processed by this reservoir might be zero
                    temporalPredecessor.wSums[reservoirIdx]         *= multipleCurrentM / temporalPredecessor.sampleNums[reservoirIdx];
                    temporalPredecessor.sampleNums[reservoirIdx]    = multipleCurrentM;
                }
            }

            // Combine to single reservoir
            Reservoir combined(current.outputSamples.size());
            combined.cameraRay                              = current.cameraRay;
            combined.hitInfo                                = current.hitInfo;
            std::array<Reservoir, 2ULL> pixelAndPredecessor = { current, temporalPredecessor };
            Reservoir::combineBiased(pixelAndPredecessor, combined, features); // Samples from temporal predecessor should be visible, no need to do unbiased combination
            reservoirGrid[y][x]                             = combined;
        }
    }
}

float generalisedBalanceHeuristic(const LightSample& sample, const std::vector<Reservoir>& allPixels,
                                  const Ray& primaryRay, const HitInfo& primaryHitInfo,
                                  const Features& features) {
    // Redundant computation in denominator, but sufficient for a quick and dirty prototype
    float numerator     = targetPDF(sample, primaryRay, primaryHitInfo, features);
    float denominator   = std::numeric_limits<float>::min();
    for (const Reservoir& pixel : allPixels) { denominator += targetPDF(sample, pixel.cameraRay, pixel.hitInfo, features); }
    return numerator / denominator;
}

float arbitraryUnbiasedContributionWeightReciprocal(const LightSample& sample, const Reservoir& pixel, const Scene& scene,
                                                    size_t sampleIdx,
                                                    const Features& features) {
    float targetPdfValue = targetPDF(sample, pixel.cameraRay, pixel.hitInfo, features);
    if (targetPdfValue == 0.0f) { return 0.0f; } // If target function value is zero, theoretical normalised PDF would also be zero

    // Compute mock unbiased contribution weight
    float mockSampleWeight  = (1.0f / static_cast<float>(features.initialLightSamples)) * // Samples initially generated
                              targetPdfValue *
                              (1.0f / scene.lights.size()); // Samples all generated via uniform light sampling, so equal original PDF
    float arbitraryWeight   = (1.0f / targetPdfValue) *
                              (pixel.wSums[sampleIdx] - pixel.chosenSampleWeights[sampleIdx] + mockSampleWeight); // Emulate replacing weight of chosen sample with the given sample
    return (1.0f / arbitraryWeight);
}

ReservoirGrid renderReSTIR(std::shared_ptr<ReservoirGrid> previousFrameGrid,
                           const Scene& scene, const Trackball& camera,
                           const BvhInterface& bvh, Screen& screen,
                           const Features& features) {
    ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);
    if (features.temporalReuse && previousFrameGrid)    { temporalReuse(reservoirGrid, *previousFrameGrid.get(), bvh, screen, features); }
    if (features.spatialReuse)                          { spatialReuse(reservoirGrid, bvh, screen, features); }

    // Final shading
    glm::ivec2 windowResolution = screen.resolution();
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
    }

    // Return current frame's final grid for temporal reuse
    return reservoirGrid;
}

void renderRMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    ReservoirGrid reservoirGrid = genInitialSamples(scene, camera, bvh, screen, features);
    glm::ivec2 windowResolution = screen.resolution();

    progressbar progressbar(windowResolution.y);
    std::cout << "Rendering with R-MIS..." << std::endl;
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

            // Apply tone mapping and set final pixel color
            if (features.enableToneMapping) { finalColor = exposureToneMapping(finalColor, features); }
            screen.setPixel(x, y, finalColor);

        }
        #pragma omp critical
        progressbar.update();
    }
    std::cout << std::endl;
}

void renderROMIS(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features) {
    constexpr int32_t TOTAL_NEIGHBOURS = 9; // TODO: Make this adjustable
    glm::ivec2 windowResolution = screen.resolution();
    MatrixGrid techniqueMatrices(windowResolution.y,        std::vector<Eigen::MatrixXf>(windowResolution.x, Eigen::MatrixXf(TOTAL_NEIGHBOURS, TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsRed(windowResolution.y,   std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsGreen(windowResolution.y, std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf(TOTAL_NEIGHBOURS)));
    VectorGrid contributionVectorsBlue(windowResolution.y,  std::vector<Eigen::VectorXf>(windowResolution.x, Eigen::VectorXf(TOTAL_NEIGHBOURS)));

    std::cout   << "Rendering with R-OMIS..."   << std::endl
                << "Sample generation..."       << std::endl;
    for (uint32_t iteration = 0U; iteration < features.maxIterationsROMIS; iteration++) {
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
                            contributionVectorsRed[y][x](rowIdx)    = sampleColor.r * scaleColVecConst;
                            contributionVectorsGreen[y][x](rowIdx)  = sampleColor.g * scaleColVecConst;
                            contributionVectorsBlue[y][x](rowIdx)   = sampleColor.b * scaleColVecConst;
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
            const Eigen::VectorXf integralComponentsRed     = tecMatrix.colPivHouseholderQr().solve(contribVecRed);
            const Eigen::VectorXf integralComponentsGreen   = tecMatrix.colPivHouseholderQr().solve(contribVecGreen);
            const Eigen::VectorXf integralComponentsBlue    = tecMatrix.colPivHouseholderQr().solve(contribVecBlue);

            // Compute final color as sum of components
            glm::vec3 finalColor(0.0f);
            for (int32_t row = 0; row < TOTAL_NEIGHBOURS; row++) {
                finalColor.r += integralComponentsRed(row);
                finalColor.g += integralComponentsGreen(row);
                finalColor.b += integralComponentsBlue(row);
            }
            screen.setPixel(x, y, finalColor);
        }
        #pragma omp critical
        progressbarPixels.update();
    }
    std::cout << std::endl;
}
