#include "shading.h"
#include "utils.h"

#include <cmath>
#include <glm/geometric.hpp>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo) {
    if (!features.enableShading) { return hitInfo.material.kd; }

    // Diffuse parameters
    glm::vec3 intersectionPos   = ray.origin + (ray.t * ray.direction);
    glm::vec3 L                 = glm::normalize(lightPosition - intersectionPos);
    float dotNL                 = glm::dot(hitInfo.normal, L);
    glm::vec3 diffuseColor      = diffuseAlbedo(hitInfo, features);

    // Shading terms and final return
    glm::vec3 diffuse   = lightColor * diffuseColor * dotNL;
    diffuse             = glm::any(glm::isnan(diffuse)) ? glm::vec3(0.0f) : diffuse;
    return diffuse;
}

const Ray computeReflectionRay(Ray ray, HitInfo hitInfo) {
    glm::vec3 incidentPoint     = ray.origin + (ray.t * ray.direction);
    glm::vec3 normL             = glm::normalize(-ray.direction);
    glm::vec3 reflectionVector  = 2.0f * glm::dot(normL, hitInfo.normal) * hitInfo.normal - normL;
    Ray reflectionRay {incidentPoint + (REFLECTION_EPSILON * reflectionVector), reflectionVector, std::numeric_limits<float>::max()};
    return reflectionRay;
}