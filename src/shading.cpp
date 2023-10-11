#include "shading.h"
#include "utils.h"

#include <cmath>
#include <glm/geometric.hpp>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo) {
    if (!features.enableShading) { return hitInfo.material.kd; }

    // Diffuse parameters
    glm::vec3 diffuseColor      = diffuseAlbedo(hitInfo, features);
    glm::vec3 intersectionPos   = ray.origin + (ray.t * ray.direction);
    glm::vec3 L                 = glm::normalize(lightPosition - intersectionPos);
    float dotNL                 = glm::dot(hitInfo.normal, L);

    // Early exit if light is behind point
    if (dotNL < 0.0f) { return glm::vec3(0.0f); }

    // Specular parameters
    glm::vec3 V                 = glm::normalize(ray.origin - intersectionPos);
    glm::vec3 R                 = glm::normalize(2.0f * dotNL * hitInfo.normal - L);
    float cosTheta              = glm::dot(R, V);

    // Shading terms and final return
    glm::vec3 diffuse   = lightColor    * diffuseColor         * dotNL;
    glm::vec3 specular  = lightColor    * hitInfo.material.ks  * std::pow(cosTheta, hitInfo.material.shininess);
    diffuse             = glm::any(glm::isnan(diffuse))     ? glm::vec3(0.0f) : diffuse;
    specular            = glm::any(glm::isnan(specular))    ? glm::vec3(0.0f) : specular;
    return diffuse + specular;
}

const Ray computeReflectionRay(Ray ray, HitInfo hitInfo) {
    glm::vec3 incidentPoint     = ray.origin + (ray.t * ray.direction);
    glm::vec3 normL             = glm::normalize(-ray.direction);
    glm::vec3 reflectionVector  = 2.0f * glm::dot(normL, hitInfo.normal) * hitInfo.normal - normL;
    Ray reflectionRay {incidentPoint + (REFLECTION_EPSILON * reflectionVector), reflectionVector, std::numeric_limits<float>::max()};
    return reflectionRay;
}