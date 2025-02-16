#pragma once
#ifndef _RENDER_UTILS_H_
#define _RENDER_UTILS_H_
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <Eigen/Dense>
DISABLE_WARNINGS_POP()

#include <framework/trackball.h>

#include <scene/scene.h>
#include <rendering/reservoir.h>
#include <rendering/screen.h>


enum class Color {
    Red = 0,
    Green,
    Blue
};

using PrimaryHitGrid    = std::vector<std::vector<RayHit>>;
using MatrixGrid        = std::vector<std::vector<Eigen::MatrixXf>>;
using VectorGrid        = std::vector<std::vector<Eigen::VectorXf>>;
using PixelGrid         = std::vector<std::vector<glm::vec3>>;

// Common
PrimaryHitGrid genPrimaryRayHits(const Scene& scene, const Trackball& camera, const EmbreeInterface& embreeInterface, const Screen& screen, const Features& features);
ReservoirGrid genInitialSamples(const PrimaryHitGrid& primaryHits, const Scene& scene, const EmbreeInterface& embreeInterface, const Features& features, const glm::ivec2& windowResolution);
glm::vec3 finalShading(const Reservoir& reservoir, const Ray& primaryRay, const EmbreeInterface& embreeInterface, const Features& features);
void combineToScreen(Screen& screen, const PixelGrid& finalPixelColors, const Features& features);

// ReSTIR-specific
void spatialReuse(ReservoirGrid& reservoirGrid, const EmbreeInterface& embreeInterface, const Screen& screen, const Features& features);
void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid, const EmbreeInterface& embreeInterface,
                   Screen& screen, const Features& features);

// R-MIS-specific
float generalisedBalanceHeuristic(const LightSample& sample, const std::vector<Reservoir>& allPixels,
                                  const Ray& primaryRay, const HitInfo& primaryHitInfo,
                                  const Features& features);

// R-OMIS-specific
void visualiseAlphas(const MatrixGrid& techniqueMatrices,
                     const VectorGrid& contributionVectorsRed,
                     const VectorGrid& contributionVectorsGreen,
                     const VectorGrid& contributionVectorsBlue,
                     const glm::ivec2& windowResolution, const Features& features);
float arbitraryUnbiasedContributionWeightReciprocal(const LightSample& sample, const Reservoir& pixel, const Scene& scene,
                                                    size_t sampleIdx,
                                                    const Features& features);
inline Eigen::VectorXf solveSystem(const Eigen::MatrixXf& A, const Eigen::VectorXf& b) { return A.completeOrthogonalDecomposition().solve(b); }


#endif // _RENDER_UTILS_H_
