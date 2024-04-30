#include "embree_interface.h"

#include <utils/utils.h>

#include <format>
#include <stdexcept>

EmbreeInterface::EmbreeInterface(const Scene& scene) {
    // Create device
    m_device = rtcNewDevice(NULL);
    if (!m_device) { std::runtime_error(std::format("Error {}: Cannot create device ", magic_enum::enum_name(rtcGetDeviceError(NULL)))); }
    rtcSetDeviceErrorFunction(m_device, errorFunction, NULL);

    // Load all geometry to Embree scene
    m_scene = rtcNewScene(m_device);
    for (const Mesh& mesh : scene.meshes) {
        // Create and populate buffers to house vertex and triangle data
        RTCGeometry geom    = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE);
        rtcSetGeometryVertexAttributeCount(geom, 1);
        glm::vec3* positions    = (glm::vec3*)  rtcSetNewGeometryBuffer(geom,   RTC_BUFFER_TYPE_VERTEX,             0,  RTC_FORMAT_FLOAT3,  sizeof(glm::vec3),  mesh.vertices.size());
        glm::vec2* texCoords    = (glm::vec2*)  rtcSetNewGeometryBuffer(geom,   RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,   0,  RTC_FORMAT_FLOAT2,  sizeof(glm::vec2),  mesh.vertices.size());
        glm::uvec3* indices     = (glm::uvec3*) rtcSetNewGeometryBuffer(geom,   RTC_BUFFER_TYPE_INDEX,              0,  RTC_FORMAT_UINT3,   sizeof(glm::uvec3), mesh.triangles.size());
        populateVertexDataBuffers(positions, texCoords, mesh.vertices);
        populateIndexBuffer(indices, mesh.triangles);

        // Commit geometry and store its ID for material info lookup
        rtcCommitGeometry(geom);
        uint32_t geomId             = rtcAttachGeometry(m_scene, geom);
        m_meshToMaterial[geomId]    = mesh.material;
        rtcReleaseGeometry(geom);
    }
    rtcCommitScene(m_scene);
}

EmbreeInterface::~EmbreeInterface() {
    rtcReleaseScene(m_scene);
    rtcReleaseDevice(m_device);
}

void EmbreeInterface::populateVertexDataBuffers(glm::vec3* positionBuffer, glm::vec2* texCoordBuffer,
                                                const std::vector<Vertex>& vertices) {
    for (size_t vertexIdx = 0ULL; vertexIdx < vertices.size(); vertexIdx++) {
        positionBuffer[vertexIdx]   = vertices[vertexIdx].position;
        texCoordBuffer[vertexIdx]   = vertices[vertexIdx].texCoord;
    }
}

void EmbreeInterface::populateIndexBuffer(glm::uvec3* indexBuffer, const std::vector<glm::uvec3>& indices) {
    for (size_t triangleIdx = 0ULL; triangleIdx < indices.size(); triangleIdx++) {
        indexBuffer[triangleIdx] = indices[triangleIdx];
    }
}
