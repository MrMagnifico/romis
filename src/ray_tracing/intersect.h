#pragma once
#include <framework/ray.h>

#include <utils/common.h>

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2);
bool intersectRayWithPlane(const Plane& plane, Ray& ray);
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p);


bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo);

bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo);

bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray);
