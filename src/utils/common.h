#pragma once
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

#include <framework/ray.h>
#include <framework/mesh.h>

#include <cereal/archives/json.hpp>
#include <cereal/types/common.hpp>
#include <utils/magic_enum.hpp>

enum class DrawMode {
    Filled,
    Wireframe
};

enum class ViewMode {
    Rasterization = 0,
    RayTraced
};

enum class RayTraceMode {
    ReSTIR = 0,
    RMIS,
    ROMIS
};

enum class MISWeightRMIS {
    Equal = 0,
    Balance
};

struct HitInfo {
    glm::vec3 normal;
    glm::vec3 barycentricCoord;
    glm::vec2 texCoord;
    Material material;
};

struct RayHit {
    Ray ray;
    HitInfo hit;
};

struct Plane {
    float D = 0.0f;
    glm::vec3 normal { 0.0f, 1.0f, 0.0f };
};

struct AxisAlignedBox {
    glm::vec3 lower { 0.0f };
    glm::vec3 upper { 1.0f };
};

struct Sphere {
    glm::vec3 center { 0.0f };
    float radius = 1.0f;
    Material material;
};

struct PointLight {
    glm::vec3 position;
    glm::vec3 color;
};

struct SegmentLight {
    glm::vec3 endpoint0, endpoint1; // Positions of endpoints
    glm::vec3 color0, color1;       // Color of endpoints
};

struct ParallelogramLight {
    // A parallelogram light (see figure 3.14 of chapter 13.4.2 of Fundamentals of CG 4th Edition)
    glm::vec3 v0;               // v0
    glm::vec3 edge01, edge02;   // Edges from v0 to v1, and from v0 to v2
    glm::vec3 color0, color1, color2, color3;
};

struct Features {
    // Feature flags
    bool enableShading          = true;
    bool enableRecursive        = false;
    bool enableHardShadow       = true;
    bool enableSoftShadow       = true;
    bool enableNormalInterp     = true;
    bool enableTextureMapping   = true;
    bool enableAccelStructure   = true;

    // Base render parameters
    uint32_t maxReflectionRecursion = 5U;

    // Shared R-MIS/ReSTIR feature flag(s) and parameter(s)
    RayTraceMode rayTraceMode           = RayTraceMode::ROMIS;
    bool initialSamplesVisibilityCheck  = false;
    uint32_t numSamplesInReservoir      = 2U;
    uint32_t initialLightSamples        = 32U;
    uint32_t numNeighboursToSample      = 5U;
    uint32_t spatialResampleRadius      = 10U;

    // R-MIS/R-OMIS parameter(s)
    uint32_t maxIterationsMIS       = 5U;
    MISWeightRMIS misWeightRMIS     = MISWeightRMIS::Equal;
    bool useProgressiveROMIS        = false;
    uint32_t progressiveUpdateMod   = 1U;
    bool saveAlphasVisualisation    = true;

    // ReSTIR feature flags
    bool unbiasedCombination            = false;
    bool spatialReuse                   = true;
    bool spatialReuseVisibilityCheck    = false;
    bool temporalReuse                  = true;

    // ReSTIR parameters
    uint32_t spatialResamplingPasses    = 2U;
    uint32_t temporalClampM             = 20U;

    // Misc parameters
    bool enableToneMapping  = true;
    float gamma             = 1.0f;
    float exposure          = 1.5f;

    template<class Archive>
    void serialize(Archive& archive) const {
        archive(CEREAL_NVP(enableShading), CEREAL_NVP(enableRecursive), CEREAL_NVP(enableHardShadow), CEREAL_NVP(enableSoftShadow), CEREAL_NVP(enableNormalInterp), CEREAL_NVP(enableTextureMapping), CEREAL_NVP(enableAccelStructure),
                CEREAL_NVP(maxReflectionRecursion),
                CEREAL_NVP(rayTraceMode), CEREAL_NVP(initialSamplesVisibilityCheck), CEREAL_NVP(numSamplesInReservoir), CEREAL_NVP(initialLightSamples), CEREAL_NVP(numNeighboursToSample), CEREAL_NVP(spatialResampleRadius),
                CEREAL_NVP(maxIterationsMIS), CEREAL_NVP(misWeightRMIS), CEREAL_NVP(useProgressiveROMIS), CEREAL_NVP(progressiveUpdateMod), CEREAL_NVP(saveAlphasVisualisation),
                CEREAL_NVP(unbiasedCombination), CEREAL_NVP(spatialReuse), CEREAL_NVP(spatialReuseVisibilityCheck), CEREAL_NVP(temporalReuse),
                CEREAL_NVP(spatialResamplingPasses), CEREAL_NVP(temporalClampM),
                CEREAL_NVP(enableToneMapping), CEREAL_NVP(gamma), CEREAL_NVP(exposure)); 
    }
};