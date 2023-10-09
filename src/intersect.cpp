#include "interpolate.h"
#include "intersect.h"
#include "utils.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <limits>


Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    glm::vec3 e0        = v0 - v2;
    glm::vec3 e1        = v1 - v2;
    glm::vec3 normal    = glm::normalize(glm::cross(e0, e1));
    float D             = glm::dot(normal, v0);
    return {D, normal};
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray) {
    float dirNormDot    = glm::dot(plane.normal, ray.direction);
    if (zeroWithinEpsilon(dirNormDot)) { return false; } // Scenario where line lies on plane considered not an intersection
    ray.t               = (plane.D - glm::dot(ray.origin, plane.normal)) / dirNormDot;
    return true;
}

bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p) {
    glm::vec3 barycentricCoords = computeBarycentricCoord(v0, v1, v2, p);
    return inRangeInclusive(barycentricCoords[0], 0.0f, 1.0f) &&
           inRangeInclusive(barycentricCoords[1], 0.0f, 1.0f) &&
           inRangeInclusive(barycentricCoords[2], 0.0f, 1.0f);
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo) {
    float tOld                  = ray.t;
    Plane plane                 = trianglePlane(v0, v1, v2);
    if (!intersectRayWithPlane(plane, ray)) { return false; }
    glm::vec3 intersectionPoint = ray.origin + (ray.t * ray.direction);
    if (!pointInTriangle(v0, v1, v2, plane.normal, intersectionPoint) ||
        tOld < ray.t || ray.t <= 0.0f) {
        ray.t = tOld;
        return false;
    }

    // Set normal and return
    hitInfo.normal = plane.normal;
    return true;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo) {
    // Quadratic equation parameters
    glm::vec3 originTranslated  = ray.origin - sphere.center;
    float A                     = glm::dot(ray.direction, ray.direction);
    float B                     = 2.0f * glm::dot(originTranslated, ray.direction);
    float C                     = glm::dot(originTranslated, originTranslated) - (sphere.radius * sphere.radius);

    // Determine no. of solutions from discriminant
    float tNew;
    float discriminant = (B * B) - (4.0f * A * C);
    if (discriminant < 0.0f)                    { return false; }
    else if (zeroWithinEpsilon(discriminant))   { tNew = -B / (2.0f * A); }
    else                                        { tNew = (-B - std::sqrt(discriminant)) / (2.0f * A); }

    // Validity test
    if (ray.t < tNew || tNew <= 0.0f) { return false; }
    ray.t = tNew;
    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray) {
    // Compute t values
    float xMin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float xMax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float yMin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float yMax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float zMin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float zMax = (box.upper.z - ray.origin.z) / ray.direction.z;

    // Compute in/out t's
    float xIn   = std::min(xMin, xMax);
    float xOut  = std::max(xMin, xMax);
    float yIn   = std::min(yMin, yMax);
    float yOut  = std::max(yMin, yMax);
    float zIn   = std::min(zMin, zMax);
    float zOut  = std::max(zMin, zMax);

    // Compute global entry and exit t's
    float tIn   = std::max(std::max(xIn, yIn), zIn);
    float tOut  = std::min(std::min(xOut, yOut), zOut);

    // Verify intersection and earliness
    if (tIn > tOut  || tOut < 0.0f ||
        ray.t < tIn || tIn <= 0.0f) { return false; }
    ray.t = tIn;
    return true;
}
