#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p) {
    // Define relevant edge vectors
    glm::vec3 e0 = v1 - v0;
    glm::vec3 e1 = v2 - v0;
    glm::vec3 e2 = p - v0;

    // Compute relevant dot products
    float d00 = glm::dot(e0, e0);
    float d01 = glm::dot(e0, e1);
    float d11 = glm::dot(e1, e1);
    float d20 = glm::dot(e2, e0);
    float d21 = glm::dot(e2, e1);

    // Compute barycentric coordinates
    float denom = (d00 * d11) - (d01 * d01);
    float beta  = (d11 * d20 - d01 * d21) / denom;
    float gamma = (d00 * d21 - d01 * d20) / denom;
    return glm::vec3(1.0f - (beta + gamma), beta, gamma);
}

glm::vec3 interpolateNormal(const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord) {
    return (n0 * barycentricCoord[0]) + (n1 * barycentricCoord[1]) + (n2 * barycentricCoord[2]);
}

glm::vec2 interpolateTexCoord(const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord) {
    return (t0 * barycentricCoord[0]) + (t1 * barycentricCoord[1]) + (t2 * barycentricCoord[2]);
}
