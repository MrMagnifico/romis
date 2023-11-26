#include "scene.h"
#include <cmath>
#include <iostream>

std::vector<ParallelogramLight> regularLightGrid(glm::vec3 startPos, glm::ivec2 counts, glm::vec3 edge01, glm::vec3 edge02,
                                                 glm::vec3 color,
                                                 float emptySpacePercentage) {
    std::vector<ParallelogramLight> lights;
    glm::vec3 spaceEdge01   = edge01 / static_cast<float>(counts[0]);
    glm::vec3 spaceEdge02   = edge02 / static_cast<float>(counts[1]);
    glm::vec3 lightEdge01   = (edge01 * (1.0f - emptySpacePercentage)) / static_cast<float>(counts[0]);
    glm::vec3 lightEdge02   = (edge02 * (1.0f - emptySpacePercentage)) / static_cast<float>(counts[1]);
    for (int xLight = 0; xLight < counts.x; xLight++) {
        for (int yLight = 0; yLight < counts.y; yLight++) {
            glm::vec3 lightOrigin = startPos + (spaceEdge01 * static_cast<float>(xLight)) + (spaceEdge02 * static_cast<float>(yLight));
            lights.emplace_back(ParallelogramLight {
                .v0 = lightOrigin,
                .edge01 = lightEdge01,
                .edge02 = lightEdge02,
                .color0 = color,
                .color1 = color,
                .color2 = color,
                .color3 = color,
            });
        }
    }
    return lights;
}

void constructNightClubLights(Scene& scene) {
    glm::vec3 startingVertex, lightEdge01, lightEdge02, lightColor, interLightDist;
    glm::ivec2 counts           = glm::ivec2(16, 16);
    float freeSpacePercentage   = 0.30f;

    // Ceiling
    // startingVertex      = glm::vec3(8.576f, 6.550f, -7.6458f);
    // lightEdge01         = glm::vec3(-16.0f, 0.0f, 0.0f);
    // lightEdge02         = glm::vec3(0.0f, 0.0f, 12.0f);
    // lightColor          = glm::vec3(1.0f);
    // std::vector<ParallelogramLight> ceilLights = regularLightGrid(startingVertex, counts, lightEdge01, lightEdge02, lightColor, freeSpacePercentage);
    // scene.lights.insert(scene.lights.end(), ceilLights.begin(), ceilLights.end());

    // Left wall
    // startingVertex      = glm::vec3(9.4f, 6.4f, -9.1f);
    // lightEdge01         = glm::vec3(0.0f, 0.0f, 17.0f);
    // lightEdge02         = glm::vec3(0.0f, -6.0f, 0.0f);
    // lightColor          = glm::vec3(1.0f);
    // std::vector<ParallelogramLight> leftWallLights = regularLightGrid(startingVertex, counts, lightEdge01, lightEdge02, lightColor, freeSpacePercentage);
    // scene.lights.insert(scene.lights.end(), leftWallLights.begin(), leftWallLights.end());

    // Right wall
    startingVertex      = glm::vec3(-8.7f, 6.4f, -9.1f);
    lightEdge01         = glm::vec3(0.0f, 0.0f, 17.0f);
    lightEdge02         = glm::vec3(0.0f, -6.0f, 0.0f);
    lightColor          = glm::vec3(0.65f);
    std::vector<ParallelogramLight> rightWallLights = regularLightGrid(startingVertex, counts, lightEdge01, lightEdge02, lightColor, freeSpacePercentage);
    scene.lights.insert(scene.lights.end(), rightWallLights.begin(), rightWallLights.end());

    // Back wall
    startingVertex      = glm::vec3(9.2f, 6.4f, 8.6f);
    lightEdge01         = glm::vec3(-17.0f, 0.0f, 0.0f);
    lightEdge02         = glm::vec3(0.0f, -6.0f, 0.0f);
    lightColor          = glm::vec3(0.4f);
    std::vector<ParallelogramLight> backWallLights = regularLightGrid(startingVertex, counts, lightEdge01, lightEdge02, lightColor, freeSpacePercentage);
    scene.lights.insert(scene.lights.end(), backWallLights.begin(), backWallLights.end());
}

Scene loadScenePrebuilt(SceneType type, const std::filesystem::path& dataDir) {
    Scene scene;
    scene.type = type;
    switch (type) {
    case SingleTriangle: {
        // Load a 3D model with a single triangle
        auto subMeshes = loadMesh(dataDir / "triangle.obj");
        subMeshes[0].material.kd = glm::vec3(1.0f);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    case Cube: {
        // Load a 3D model of a cube with 12 triangles
        auto subMeshes = loadMesh(dataDir / "cube.obj");
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        // scene.lights.push_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
        scene.lights.emplace_back(SegmentLight {
            .endpoint0 = glm::vec3(1.5f, 0.5f, -0.6f),
            .endpoint1 = glm::vec3(-1, 0.5f, -0.5f),
            .color0 = glm::vec3(0.9f, 0.2f, 0.1f), // Red-ish
            .color1 = glm::vec3(0.2f, 1, 0.3f) // Green-ish
        });
    } break;
    case CubeTextured: {
        auto subMeshes = loadMesh(dataDir / "cube-textured.obj");
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(-1.0, 1.5, -1.0), glm::vec3(1) });
    } break;
    case CornellBox: {
        // Load a 3D model of a Cornell Box
        auto subMeshes = loadMesh(dataDir / "CornellBox-Mirror-Rotated.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(0, 0.58f, 0), glm::vec3(1) }); // Light at the top of the box
    } break;
    case CornellBoxParallelogramLight: {
        // Load a 3D model of a Cornell Box
        auto subMeshes = loadMesh(dataDir / "CornellBox-Mirror-Rotated.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        // Light at the top of the box.
        scene.lights.emplace_back(ParallelogramLight {
            .v0 = glm::vec3(-0.2f, 0.5f, 0),
            .edge01 = glm::vec3(0.4f, 0, 0),
            .edge02 = glm::vec3(0.0f, 0.0f, 0.4f),
            .color0 = glm::vec3(1.0f, 1.0f, 1.0f), // White
            .color1 = glm::vec3(0.5f, 0.5f, 0.5f), // Grey
            .color2 = glm::vec3(0.5f, 0.5f, 0.5f), // Grey
            .color3 = glm::vec3(1.0f, 1.0f, 1.0f), // White
        });
    } break;
    case CornellNightClub: {
        auto subMeshes = loadMesh(dataDir / "cornell-nightclub.obj", false);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        constructNightClubLights(scene);
    } break;
    case Monkey: {
        // Load a 3D model of a Monkey
        auto subMeshes = loadMesh(dataDir / "monkey.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
        scene.lights.emplace_back(PointLight { glm::vec3(1, -1, -1), glm::vec3(1) });
    } break;
    case Teapot: {
        // Load a 3D model of a Teapot
        auto subMeshes = loadMesh(dataDir / "teapot.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    case Dragon: {
        // Load a 3D model of a Dragon
        auto subMeshes = loadMesh(dataDir / "dragon.obj", true);
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        scene.lights.emplace_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
    } break;
    case Spheres: {
        scene.spheres.push_back(Sphere { glm::vec3(3.0f, -2.0f, 10.2f), 1.0f, Material { glm::vec3(0.8f, 0.2f, 0.2f) } });
        scene.spheres.push_back(Sphere { glm::vec3(-2.0f, 2.0f, 4.0f), 2.0f, Material { glm::vec3(0.6f, 0.8f, 0.2f) } });
        scene.spheres.push_back(Sphere { glm::vec3(0.0f, 0.0f, 6.0f), 0.75f, Material { glm::vec3(0.2f, 0.2f, 0.8f) } });
        scene.lights.emplace_back(PointLight { glm::vec3(3, 0, 3), glm::vec3(15) });
    } break;
    case Custom: {
        // === Replace custom.obj by your own 3D model (or call your 3D model custom.obj) ===
        auto subMeshes = loadMesh(dataDir / "custom.obj");
        std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
        // === CHANGE THE LIGHTING IF DESIRED ===
        scene.lights.emplace_back(PointLight { glm::vec3(-1, 1, -1), glm::vec3(1) });
        // Spherical light: position, radius, color
        // scene.lights.push_back(SphericalLight{ glm::vec3(0, 1.5f, 0), 0.2f, glm::vec3(1) });
    } break;
    };

    return scene;
}

Scene loadSceneFromFile(const std::filesystem::path& path, const std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight>>& lights) {
    Scene scene;
    scene.lights    = std::move(lights);
    auto subMeshes  = loadMesh(path);
    std::move(std::begin(subMeshes), std::end(subMeshes), std::back_inserter(scene.meshes));
    return scene;
}
