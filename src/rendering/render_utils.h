#pragma once
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

using MatrixGrid            = std::vector<std::vector<Eigen::MatrixXf>>;
using VectorGrid            = std::vector<std::vector<Eigen::VectorXf>>;
using PixelGrid             = std::vector<std::vector<glm::vec3>>;
using ResampleIndicesGrid   = std::vector<std::vector<std::vector<glm::ivec2>>>;

// Common
ReservoirGrid genInitialSamples(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, const Screen& screen, const Features& features);
glm::vec3 finalShading(const Reservoir& reservoir, const Ray& primaryRay, const BvhInterface& bvh, const Features& features);
void combineToScreen(Screen& screen, const PixelGrid& finalPixelColors, const Features& features);
std::vector<glm::ivec2> candidateIndices(int32_t x, int32_t y,
                                         const glm::ivec2& windowResolution, const Features& features);
ResampleIndicesGrid generateResampleIndicesGrid(const glm::ivec2& windowResolution, const Features& features);

// ReSTIR-specific
void spatialReuse(ReservoirGrid& reservoirGrid, const BvhInterface& bvh, const Screen& screen, const Features& features);
void temporalReuse(ReservoirGrid& reservoirGrid, ReservoirGrid& previousFrameGrid, const BvhInterface& bvh,
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
