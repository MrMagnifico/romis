#pragma once
#ifndef _NEIGHBOUR_SELECTION_H_
#define _NEIGHBOUR_SELECTION_H_

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
DISABLE_WARNINGS_POP()

#include <utils/common.h>

#include <vector>

using ResampleIndicesGrid = std::vector<std::vector<std::vector<glm::ivec2>>>;

enum class NeighbourSelectionStrategy {
    Random = 0,
    Similar,
    Dissimilar,
    EqualSimilarDissimilar
};

std::vector<glm::ivec2> indicesRandom(int32_t x, int32_t y,
                                      const glm::ivec2& windowResolution, const Features& features);
ResampleIndicesGrid generateResampleIndicesGrid(const glm::ivec2& windowResolution, const Features& features);

#endif // _NEIGHBOUR_SELECTION_H_
