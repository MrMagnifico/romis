#include "bounding_volume_hierarchy.h"

#include <ray_tracing/interpolate.h>
#include <ray_tracing/intersect.h>
#include <scene/scene.h>
#include <scene/texture.h>
#include <ui/draw.h>

#include <algorithm>
#include <glm/glm.hpp> 
#include <queue>


glm::vec3 Primitive::centroid() const {
    return glm::vec3((v0.position.x + v1.position.x + v2.position.x) / 3.0f,
                     (v0.position.y + v1.position.y + v2.position.y) / 3.0f,
                     (v0.position.z + v1.position.z + v2.position.z) / 3.0f);
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene) {
    std::vector<Primitive> allPrimitives    = buildPrimitives();
    m_rootIdx                               = constructRecursive(allPrimitives, 0);
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const { return m_numLevels; }

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const { return m_leafIndices.size(); }

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level) {
    // BFS until we find the target level and acquire its nodes
    int currentLevel = 0;
    std::queue<const Node*> currentLevelNodes;
    currentLevelNodes.push(&m_nodes[m_rootIdx]);
    while (currentLevel++ < level) {
        std::vector<const Node*> nextLevel;
        while (!currentLevelNodes.empty()) {
            // Acquire next node and remove it from the queue
            const Node* node = currentLevelNodes.front();
            currentLevelNodes.pop();

            // If the node is not a leaf, add its children to the set of next level items
            if (node->isLeaf()) { continue; }
            nextLevel.push_back(&m_nodes[node->leftChild()]);
            nextLevel.push_back(&m_nodes[node->rightChild()]);
        }
        for (const Node* nextLevelNode : nextLevel) { currentLevelNodes.push(nextLevelNode); }
    }

    // Draw all AABBs at this level
    while (!currentLevelNodes.empty()) {
        // Acquire next node and remove it from the queue
        const Node* node = currentLevelNodes.front();
        currentLevelNodes.pop();

        // Draw AABB as semi-transparent green wireframe
        drawAABB(node->aabb, DrawMode::Wireframe, glm::vec3(0.0f, 1.0f, 0.0f), 0.5f);
    }
}


// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. Note: leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx) {
    // Obtain leaf node and view over contained triangles
    Node leafNode = m_nodes[m_leafIndices[leafIdx]];
    std::vector<Primitive> triangles(m_primitives.begin() + leafNode.primitiveOffset(),
                                     m_primitives.begin() + leafNode.primitiveOffset() + leafNode.primitiveCount());

    // Draw AABB and contained triangles
    drawAABB(leafNode.aabb, DrawMode::Wireframe, glm::vec3(1.0f, 1.0f, 1.0f), 0.5f);
    for (const Primitive& triangle : triangles) { drawTriangle(triangle.v0, triangle.v1, triangle.v2); }
}

bool BoundingVolumeHierarchy::intersectNaive(Ray& ray, HitInfo& hitInfo, const Features& features) const {
    bool hit = false;

    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                // Optional hit info
                if (features.enableNormalInterp || features.enableTextureMapping) {
                    glm::vec3 intersectionPos   = ray.origin + (ray.t * ray.direction);
                    glm::vec3 barycentricCoords = computeBarycentricCoord(v0.position, v1.position, v2.position, intersectionPos);
                    if (features.enableNormalInterp)    { hitInfo.normal    = interpolateNormal(v0.normal, v1.normal, v2.normal, barycentricCoords); }
                    if (features.enableTextureMapping)  { hitInfo.texCoord  = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, barycentricCoords); }
                }

                // Non-optional hit info
                hitInfo.material    = mesh.material;
                hit                 = true;
            }
        }
    }

    // Intersect with spheres
    for (const auto& sphere : m_pScene->spheres) { hit |= intersectRayWithShape(sphere, ray, hitInfo); }

    // Debug ray(s) and hit value return
    if (features.enableNormalInterp) { drawRay({ray.origin + (ray.t * ray.direction), hitInfo.normal, 1.0f}, NORMAL_INTERP_VIS_COLOR); }
    return hit;
}

bool BoundingVolumeHierarchy::intersectAcceleratedRecursive(const Node& currentNode, Ray& ray, HitInfo& hitInfo, const Features& features) const {
    // Check for intersection with current node and restore t-value
    float tOriginal = ray.t;
    bool hitBox     = intersectRayWithShape(currentNode.aabb, ray);
    if (!hitBox) { return false; }
    ray.t           = tOriginal;

    // Intersection test with all primitives if the current node is a leaf
    if (currentNode.isLeaf()) {
        bool hit                    = false;
        uint32_t finalIdxExclusive  = currentNode.primitiveOffset() + currentNode.primitiveCount();
        for (uint32_t primitiveIdx  = currentNode.primitiveOffset(); primitiveIdx < finalIdxExclusive; primitiveIdx++) {
            const Primitive& tri = m_primitives[primitiveIdx];
            if (intersectRayWithTriangle(tri.v0.position, tri.v1.position, tri.v2.position, ray, hitInfo)) {
                // Optional hit info
                if (features.enableNormalInterp || features.enableTextureMapping) {
                    glm::vec3 intersectionPos   = ray.origin + (ray.t * ray.direction);
                    glm::vec3 barycentricCoords = computeBarycentricCoord(tri.v0.position, tri.v1.position, tri.v2.position, intersectionPos);
                    if (features.enableNormalInterp)    { hitInfo.normal    = interpolateNormal(tri.v0.normal, tri.v1.normal, tri.v2.normal, barycentricCoords); }
                    if (features.enableTextureMapping)  { hitInfo.texCoord  = interpolateTexCoord(tri.v0.texCoord, tri.v1.texCoord, tri.v2.texCoord, barycentricCoords); }
                }

                // Non-optional hit info
                hitInfo.material    = m_pScene->meshes[tri.meshID].material;
                hit                 = true;
            }
        }

        // Debug ray(s) and hit value return
        if (features.enableNormalInterp) { drawRay({ray.origin + (ray.t * ray.direction), hitInfo.normal, 1.0f}, NORMAL_INTERP_VIS_COLOR); }
        return hit;
    } else { // Recursive calls to children if node is interior
        bool leftHit    = intersectAcceleratedRecursive(m_nodes[currentNode.leftChild()],   ray, hitInfo, features);
        bool rightHit   = intersectAcceleratedRecursive(m_nodes[currentNode.rightChild()],  ray, hitInfo, features);
        return leftHit || rightHit;
    }
}

bool BoundingVolumeHierarchy::intersectAccelerated(Ray& ray, HitInfo& hitInfo, const Features& features) const {
    return intersectAcceleratedRecursive(m_nodes[m_rootIdx], ray, hitInfo, features);
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const {
    if (!features.enableAccelStructure) { return intersectNaive(ray, hitInfo, features); }
    else                                { return intersectAccelerated(ray, hitInfo, features); }
}

uint32_t BoundingVolumeHierarchy::constructRecursive(const std::span<Primitive>& primitives, int currentLevel) {
    m_numLevels             = std::max(m_numLevels, currentLevel + 1);
    AxisAlignedBox bbNode   = boundingBox(primitives);

    // Construct leaf if we are operating with a small enough number of leaves
    if (primitives.size() <= LeafSize) {
        // Construct node data
        Node leaf = {
            .aabb = bbNode,
            .data = { static_cast<uint32_t>(m_primitives.size()) | Node::LeafBit, static_cast<uint32_t>(primitives.size()) }
        };

        // Update relevant containers and terminate
        uint32_t nodeIndex = static_cast<uint32_t>(m_nodes.size());
        m_primitives.insert(m_primitives.end(), primitives.begin(), primitives.end());
        m_leafIndices.push_back(nodeIndex);
        m_nodes.push_back(leaf);
        return nodeIndex;
    }

    // Split primitives in half along the longest axis since this is an interior node
    uint32_t splitAxis = longestAxis(bbNode);
    switch (splitAxis) {
        case 0: {
            std::sort(primitives.begin(), primitives.end(), Primitive::xAxisSort);
        } break;
        case 1: {
            std::sort(primitives.begin(), primitives.end(), Primitive::yAxisSort);
        } break;
        case 2: {
            std::sort(primitives.begin(), primitives.end(), Primitive::zAxisSort);
        } break;
        default: { // Split along x-axis by default (this is only a fail-safe and should never actually occur)
            std::sort(primitives.begin(), primitives.end(), Primitive::xAxisSort);
        }
    }

    // Recursively construct lower levels and node data
    size_t splitIndex       = primitives.size() / 2UL;
    size_t lastN            = primitives.size() - splitIndex;
    uint32_t leftChildIdx   = constructRecursive(primitives.subspan(0UL, splitIndex), currentLevel + 1);
    uint32_t rightChildIdx  = constructRecursive(primitives.subspan(splitIndex, lastN), currentLevel + 1);
    Node interior = {
        .aabb = bbNode,
        .data = { leftChildIdx, rightChildIdx }
    };

    // Insert node into node vector and return its index
    uint32_t nodeIndex = static_cast<uint32_t>(m_nodes.size());
    m_nodes.push_back(interior);
    return nodeIndex;
}

std::vector<Primitive> BoundingVolumeHierarchy::buildPrimitives() const {
    // Count the total nr. of triangles in the scene
    size_t numTriangles = 0ULL;
    for (const auto& mesh : m_pScene->meshes) { numTriangles += mesh.triangles.size(); }

    // Given the input scene, gather all triangles over which to build the BVH as a list of Primitives
    std::vector<Primitive> primitives;
    primitives.reserve(numTriangles);
    for (uint32_t meshID = 0; meshID < m_pScene->meshes.size(); meshID++) {
        const auto& mesh = m_pScene->meshes[meshID];
        for (const auto& triangle : mesh.triangles) {
            primitives.push_back(Primitive {
                .meshID = meshID,
                .v0 = mesh.vertices[triangle.x],
                .v1 = mesh.vertices[triangle.y],
                .v2 = mesh.vertices[triangle.z] });
        }
    }
    return primitives;
}

AxisAlignedBox BoundingVolumeHierarchy::boundingBox(const std::span<Primitive>& triangles) const {
    AxisAlignedBox bb = {
        .lower = glm::vec3(std::numeric_limits<float>::max()),
        .upper = glm::vec3(std::numeric_limits<float>::min())
    };

    for (const Primitive& triangle : triangles) {
        std::array<glm::vec3, 3ULL> verticesPositions = { triangle.v0.position, triangle.v1.position, triangle.v2.position };
        for (const glm::vec3& vertexPos : verticesPositions) {
            bb.lower.x = std::min(bb.lower.x, vertexPos.x);
            bb.lower.y = std::min(bb.lower.y, vertexPos.y);
            bb.lower.z = std::min(bb.lower.z, vertexPos.z);
            bb.upper.x = std::max(bb.upper.x, vertexPos.x);
            bb.upper.y = std::max(bb.upper.y, vertexPos.y);
            bb.upper.z = std::max(bb.upper.z, vertexPos.z);
        }
    }
    return bb;
}

uint32_t BoundingVolumeHierarchy::longestAxis(const AxisAlignedBox& box) const {
    uint32_t maxAxis    = 0U;
    float maxDist       = std::numeric_limits<float>::min();
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        float axisDist = box.upper[axis] - box.lower[axis];
        if (axisDist > maxDist) {
            maxAxis = axis;
            maxDist = axisDist;
        }
    }
    return maxAxis;
}
