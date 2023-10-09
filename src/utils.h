#pragma once
#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <limits>

float constexpr ZERO_EPSILON = 1E-5F;

float zeroWithinEpsilon(float val);

float linearMap(float val, float domainMin, float domainMax, float rangeMin, float rangeMax);

template<typename T>
inline bool inRangeInclusive(T val, T low, T high) { return low <= val && val <= high; }

#endif
