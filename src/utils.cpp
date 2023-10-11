#include "utils.h"
#include "texture.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
DISABLE_WARNINGS_POP()
#include <framework/opengl_includes.h>

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

void setOpenGLMatrices(const Trackball& camera) {
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}
