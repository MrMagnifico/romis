#pragma once
#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include "common.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <limits>

float constexpr ZERO_EPSILON = 1E-5F;

float zeroWithinEpsilon(float val);

float linearMap(float val, float domainMin, float domainMax, float rangeMin, float rangeMax);

template<typename T>
inline bool inRangeInclusive(T val, T low, T high) { return low <= val && val <= high; }

glm::vec3 diffuseAlbedo(const HitInfo& hitInfo, const Features& features);

#endif
