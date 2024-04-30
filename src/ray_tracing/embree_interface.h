#pragma once
#ifndef _EMBREE_INTERFACE_H_
#define _EMBREE_INTERFACE_H_

#include <embree4/rtcore.h>

#include <scene/scene.h>

#include <unordered_map>


class EmbreeInterface {
public:
    EmbreeInterface(const Scene& scene);
    ~EmbreeInterface();

    void changeScene(const Scene& scene);
    bool anyHit(Ray& ray) const;
    bool closestHit(Ray& ray, HitInfo& hitInfo) const;

private:
    void initDevice();
    void initScene(const Scene& scene);
    void populateVertexDataBuffers(glm::vec3* positionBuffer, glm::vec3* normalBuffer, glm::vec2* texCoordBuffer,
                                   const std::vector<Vertex>& vertices);
    void populateIndexBuffer(glm::uvec3* indexBuffer, const std::vector<glm::uvec3>& indices);
    RTCRayHit constructEmbreeRay(const Ray& ray) const;

    RTCDevice m_device;
    RTCScene m_scene;
    std::unordered_map<uint32_t, Material> m_meshToMaterial;
};


#endif // _EMBREE_INTERFACE_H_
