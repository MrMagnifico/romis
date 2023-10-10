#include "utils.h"
#include "texture.h"

#include <cstdlib>

float zeroWithinEpsilon(float val) { return std::abs(val) < ZERO_EPSILON; }

float linearMap(float val, float domainMin, float domainMax, float rangeMin, float rangeMax) {
    float ratio         = (val - domainMin) / (domainMax - domainMin);
    float scaledValue   = ratio * (rangeMax - rangeMin);
    float mappedValue   = scaledValue + rangeMin;
    return mappedValue;
}

glm::vec3 diffuseAlbedo(const HitInfo& hitInfo, const Features& features) {
    return features.enableTextureMapping && hitInfo.material.kdTexture                   ?
           acquireTexel(*hitInfo.material.kdTexture.get(), hitInfo.texCoord, features)   :
           hitInfo.material.kd;
}
