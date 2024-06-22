#pragma once
#ifndef _NEIGHBOUR_SELECTION_H_
#define _NEIGHBOUR_SELECTION_H_

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
DISABLE_WARNINGS_POP()

#include <rendering/render_utils.h>
#include <utils/common.h>

#include <vector>


bool areSimilar(const RayHit& lhs, const RayHit& rhs, const Features& features);
std::vector<glm::ivec2> indicesRandom(int32_t x, int32_t y,
                                      const glm::ivec2& windowResolution, const Features& features);
std::vector<glm::ivec2> indicesSimilarity(int32_t x, int32_t y,
                                          const PrimaryHitGrid& primaryHits, const glm::ivec2& windowResolution, const Features& features);
ResampleIndicesGrid generateResampleIndicesGrid(const PrimaryHitGrid& primaryHits,
                                                const glm::ivec2& windowResolution, const Features& features);

#endif // _NEIGHBOUR_SELECTION_H_
