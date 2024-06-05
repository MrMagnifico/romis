#include "ui.h"

#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <imgui/imgui.h>
#include <nativefiledialog/nfd.h>
DISABLE_WARNINGS_POP()

#include <framework/imguizmo.h>
#include <framework/variant_helper.h>

#include <rendering/render.h>
#include <utils/magic_enum.hpp>
#include <utils/utils.h>

#include <array>
#include <iostream>
#include <memory>
#include <variant>


UiManager::UiManager(EmbreeInterface& embreeInterface, Trackball& camera, Config& config, std::optional<RayHit>& optDebugRayHit,
                     std::shared_ptr<ReservoirGrid>& previousFrameGrid, Scene& scene, SceneType& sceneType,
                     Screen& screen, ViewMode& viewMode, Window& window,
                     int& selectedLightIdx)
    : embreeInterface(embreeInterface)
    , camera(camera)
    , config(config)
    , optDebugRayHit(optDebugRayHit)
    , previousFrameGrid(previousFrameGrid)
    , scene(scene)
    , sceneType(sceneType)
    , screen(screen)
    , viewMode(viewMode)
    , window(window)
    , selectedLightIdx(selectedLightIdx)
{}

void UiManager::draw() {
    ImGui::Begin("RMIS Implementation");
    ImGui::BeginTabBar("Options");
    if (ImGui::BeginTabItem("Ray Tracing")) {
        drawRayTracingTab();
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("General")) {
        drawProjectTab();
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Misc")) {
        drawMiscTab();
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::End();
}

void UiManager::drawProjectTab() {
    drawSceneSelection();
    drawViewModeSelection();
    ImGui::Separator();
    drawFeaturesToggles();
    ImGui::Separator();
    drawCameraStats();
    ImGui::Spacing();
    ImGui::Separator();
    drawRenderToFile();
    ImGui::Spacing();
    ImGui::Separator();
    drawLightControls();
}

void UiManager::drawRayTracingTab() {
    drawRayTracingNeighbourSelectionParams();
    ImGui::Spacing();
    ImGui::Separator();
    drawRayTracingFeaturesToggles();
    ImGui::Spacing();
    ImGui::Separator();
    drawRayTracingParams();
}

void UiManager::drawMiscTab() {
    drawToneMappingControls();
}

void UiManager::drawSceneSelection() {
    constexpr auto availableScenes = magic_enum::enum_names<SceneType>();
    std::vector<const char*> availableScenesPointers;
    std::transform(std::begin(availableScenes), std::end(availableScenes), std::back_inserter(availableScenesPointers),
                   [](const auto& str) { return str.data(); });
    if (ImGui::Combo("Scene", reinterpret_cast<int*>(&sceneType), availableScenesPointers.data(), static_cast<int>(availableScenesPointers.size()))) {
        optDebugRayHit.reset();
        scene               = loadScenePrebuilt(sceneType, config.dataPath, camera, config.features);
        selectedLightIdx    = scene.lights.empty() ? -1 : 0;
        embreeInterface.changeScene(scene);
        if (optDebugRayHit) {
            HitInfo dummy {};
            embreeInterface.closestHit((*optDebugRayHit).ray, dummy);
        }
    }
}

void UiManager::drawViewModeSelection() {
    constexpr auto viewModes = magic_enum::enum_names<ViewMode>();
    std::vector<const char*> viewModesPointers;
    std::transform(std::begin(viewModes), std::end(viewModes), std::back_inserter(viewModesPointers),
                   [](const auto& str) { return str.data(); });
    ImGui::Combo("Render mode", (int*) &viewMode, viewModesPointers.data(), static_cast<int>(viewModesPointers.size()));
}

void UiManager::drawFeaturesToggles() {
    if (ImGui::CollapsingHeader("Features")) {
        ImGui::Checkbox("Shading",                  &config.features.enableShading);
        ImGui::Checkbox("Recursive(reflections)",   &config.features.enableRecursive);
        ImGui::Checkbox("Hard shadows",             &config.features.enableHardShadow);
        ImGui::Checkbox("Soft shadows",             &config.features.enableSoftShadow);
        ImGui::Checkbox("BVH",                      &config.features.enableAccelStructure);
        ImGui::Checkbox("Texture mapping",          &config.features.enableTextureMapping);
        ImGui::Checkbox("Normal interpolation",     &config.features.enableNormalInterp);
    }
}

void UiManager::drawCameraStats() {
    if (ImGui::TreeNode("Camera (read only)")) {
        auto lookAt     = camera.lookAt();
        auto position   = camera.position();
        auto rotation   = glm::degrees(camera.rotationEulerAngles());
        auto distance   = camera.distanceFromLookAt();
        ImGui::InputFloat3("Position", glm::value_ptr(position), "%0.2f", ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat3("LookAt", glm::value_ptr(lookAt), "%0.2f", ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat("Distance from look at", &distance, 0.1f, 0.1f, "%0.2f", ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat3("Rotation", glm::value_ptr(rotation), "%0.2f", ImGuiInputTextFlags_ReadOnly);
        ImGui::TreePop();
    }
}

void UiManager::drawRenderToFile() {
    if (ImGui::Button("Render to file")) {
        // Show a file picker.
        nfdchar_t* pOutPath         = nullptr;
        const nfdresult_t result    = NFD_SaveDialog("bmp", nullptr, &pOutPath);
        if (result == NFD_OKAY) {
            std::filesystem::path outPath { pOutPath };
            free(pOutPath);                     // NFD is a C API so we have to manually free the memory it allocated.
            outPath.replace_extension("bmp");   // Make sure that the file extension is *.bmp

            // Perform a new render and measure the time it took to generate the image.
            using clock                             = std::chrono::high_resolution_clock;
            const auto start                        = clock::now();
            std::optional<ReservoirGrid> maybeGrid  = renderRayTraced(previousFrameGrid, scene, camera, embreeInterface, screen, config.features);
            if (maybeGrid) { previousFrameGrid      = std::make_shared<ReservoirGrid>(maybeGrid.value()); }
            const auto end                          = clock::now();
            std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
            
            // Store the new image
            screen.writeBitmapToFile(outPath);
        }
    }
}

void UiManager::drawLightControls() {
    ImGui::Text("Lights");
    
    // Configure light selection dropdown data
    std::vector<std::string> options;
    options.push_back("None");
    for (size_t i = 0; i < scene.lights.size(); i++) { options.push_back("Light " + std::to_string(i)); }
    std::vector<const char*> optionsPointers;
    std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers), [](const auto& str) { return str.c_str(); });

    // Offset such that selectedLightIdx=-1 becomes item 0 (None).
    ++selectedLightIdx;
    ImGui::Combo("Selected light", &selectedLightIdx, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
    --selectedLightIdx;

    if (selectedLightIdx >= 0) {
        setOpenGLMatrices(camera);
        std::visit(
            make_visitor(
                [&](PointLight& light) {
                    showImGuizmoTranslation(window, camera, light.position); // 3D controls to translate light source.
                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                },
                [&](SegmentLight& light) {
                    // 3D controls to translate light source.
                    static int selectedEndpoint = 0;
                    if (selectedEndpoint == 0)  { showImGuizmoTranslation(window, camera, light.endpoint0); }
                    else                        { showImGuizmoTranslation(window, camera, light.endpoint1); }

                    const std::array<const char*, 2> endpointOptions { "Endpoint 0", "Endpoint 1" };
                    ImGui::Combo("Selected endpoint", &selectedEndpoint, endpointOptions.data(), int(endpointOptions.size()));
                    ImGui::DragFloat3("Endpoint 0", glm::value_ptr(light.endpoint0), 0.01f, -3.0f, 3.0f);
                    ImGui::DragFloat3("Endpoint 1", glm::value_ptr(light.endpoint1), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                    ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                },
                [&](ParallelogramLight& light) {
                    glm::vec3 vertex1 = light.v0 + light.edge01;
                    glm::vec3 vertex2 = light.v0 + light.edge02;

                    // 3D controls to translate light source.
                    static int selectedVertex = 0;
                    if (selectedVertex == 0)        { showImGuizmoTranslation(window, camera, light.v0); }
                    else if (selectedVertex == 1)   { showImGuizmoTranslation(window, camera, vertex1); }
                    else                            { showImGuizmoTranslation(window, camera, vertex2); }

                    const std::array<const char*, 3> vertexOptions { "Vertex 0", "Vertex 1", "Vertex 2" };
                    ImGui::Combo("Selected vertex", &selectedVertex, vertexOptions.data(), int(vertexOptions.size()));
                    ImGui::DragFloat3("Vertex 0", glm::value_ptr(light.v0), 0.01f, -3.0f, 3.0f);
                    ImGui::DragFloat3("Vertex 1", glm::value_ptr(vertex1), 0.01f, -3.0f, 3.0f);
                    light.edge01 = vertex1 - light.v0;
                    ImGui::DragFloat3("Vertex 2", glm::value_ptr(vertex2), 0.01f, -3.0f, 3.0f);
                    light.edge02 = vertex2 - light.v0;

                    ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                    ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                    ImGui::ColorEdit3("Color 2", glm::value_ptr(light.color2));
                    ImGui::ColorEdit3("Color 3", glm::value_ptr(light.color3));
                },
                [&](DiskLight& light) {
                    showImGuizmoTranslation(window, camera, light.position); // 3D controls to translate light source.
                    ImGui::DragFloat3("Position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::DragFloat3("Normal", glm::value_ptr(light.normal), 0.1f, -1.0f, 1.0f);
                    ImGui::ColorEdit3("Color", glm::value_ptr(light.color));
                    ImGui::DragFloat("Radius", &light.radius, 0.1f, 0.01f, 10.0f);
                },
                [](auto) { /* any other type of light */ }),
            scene.lights[size_t(selectedLightIdx)]);
    }


    if (ImGui::Button("Add point light")) {
        selectedLightIdx = int(scene.lights.size());
        scene.lights.emplace_back(PointLight { .position = glm::vec3(0.0f), .color = glm::vec3(1.0f) });
    }
    if (ImGui::Button("Add segment light")) {
        selectedLightIdx = int(scene.lights.size());
        scene.lights.emplace_back(SegmentLight { .endpoint0 = glm::vec3(0.0f), .endpoint1 = glm::vec3(1.0f), .color0 = glm::vec3(1, 0, 0), .color1 = glm::vec3(0, 0, 1) });
    }
    if (ImGui::Button("Add parallelogram light")) {
        selectedLightIdx = int(scene.lights.size());
        scene.lights.emplace_back(ParallelogramLight {
            .v0 = glm::vec3(0.0f),
            .edge01 = glm::vec3(1, 0, 0),
            .edge02 = glm::vec3(0, 1, 0),
            .color0 = glm::vec3(1, 0, 0),   // red
            .color1 = glm::vec3(0, 1, 0),   // green
            .color2 = glm::vec3(0, 0, 1),   // blue
            .color3 = glm::vec3(1, 1, 1)    // white
        });
    }
    if (ImGui::Button("Add disc light")) {
        selectedLightIdx = int(scene.lights.size());
        scene.lights.emplace_back(DiskLight { .position = glm::vec3(0.0f), .normal = glm::vec3(0.0f, 0.0f, 1.0f), .color = glm::vec3(1.0f), .radius = 0.5f });
    }
    if (selectedLightIdx >= 0 && ImGui::Button("Remove selected light")) {
        scene.lights.erase(std::begin(scene.lights) + selectedLightIdx);
        selectedLightIdx = -1;
    }
}

void UiManager::drawRayTracingNeighbourSelectionParams() {
    if (ImGui::CollapsingHeader("Neighbour Selection Heuristics", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Same geometry", &config.features.neighbourSameGeometry);
        ImGui::SliderFloat("Max depth difference fraction", &config.features.neighbourMaxDepthDifferenceFraction, 0.01f, 1.0f);
        ImGui::SliderAngle("Max normal angle difference", &config.features.neighbourMaxNormalAngleDifferenceRadians, 0.0f, 90.0f);
    }
}

void UiManager::drawRayTracingFeaturesToggles() {
    if (ImGui::CollapsingHeader("Features", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Common");
        ImGui::Checkbox("Initial samples - Visibility check",   &config.features.initialSamplesVisibilityCheck);

        ImGui::Spacing();
        ImGui::Separator();

        ImGui::Text("R-MIS / R-OMIS");
        ImGui::Checkbox("Save alphas visualisation", &config.features.saveAlphasVisualisation);

        ImGui::Spacing();
        ImGui::Separator();

        ImGui::Text("ReSTIR");
        ImGui::Checkbox("Use unbiased combination",             &config.features.unbiasedCombination);
        ImGui::Checkbox("Spatial reuse",                        &config.features.spatialReuse);
        ImGui::Checkbox("Spatial reuse - Visibility check",     &config.features.spatialReuseVisibilityCheck);
        ImGui::Checkbox("Temporal reuse",                       &config.features.temporalReuse);
    }
}

void UiManager::drawRayTracingParams() {
    if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Mode selection
        constexpr auto rayTraceModes = magic_enum::enum_names<RayTraceMode>();
        std::vector<const char*> rayTraceModesPointers;
        std::transform(std::begin(rayTraceModes), std::end(rayTraceModes), std::back_inserter(rayTraceModesPointers),
                       [](const auto& str) { return str.data(); });
        ImGui::Combo("Ray tracing mode", (int*) &config.features.rayTraceMode, rayTraceModesPointers.data(), static_cast<int>(rayTraceModesPointers.size()));
        ImGui::Spacing();

        // Common parameters
        ImGui::Text("Common");
        ImGui::SliderInt("Samples per reservoir",   (int*) &config.features.numSamplesInReservoir,      1, 32);
        ImGui::SliderInt("Canonical sample count",  (int*) &config.features.initialLightSamples,        1, 256);
        ImGui::SliderInt("Neighbours to sample",    (int*) &config.features.numNeighboursToSample,      0, 10);
        ImGui::SliderInt("Spatial resample radius", (int*) &config.features.spatialResampleRadius,      1, 30);

        ImGui::Spacing();
        ImGui::Separator();

        // R-MIS/R-OMIS parameters
        ImGui::Text("R-MIS / R-OMIS");
        ImGui::SliderInt("Max iterations",  (int*) &config.features.maxIterationsMIS, 1, 16);
        constexpr auto neighbourSelectionStrategies = magic_enum::enum_names<NeighbourSelectionStrategy>();
        std::vector<const char*> neighbourSelectionStrategiesPointers;
        std::transform(std::begin(neighbourSelectionStrategies), std::end(neighbourSelectionStrategies), std::back_inserter(neighbourSelectionStrategiesPointers),
                       [](const auto& str) { return str.data(); });
        ImGui::Combo("Neighbour selection strategy", (int*) &config.features.neighbourSelectionStrategy, neighbourSelectionStrategiesPointers.data(), static_cast<int>(neighbourSelectionStrategiesPointers.size()));
        constexpr auto misWeights = magic_enum::enum_names<MISWeightRMIS>();
        std::vector<const char*> misWeightPointers;
        std::transform(std::begin(misWeights), std::end(misWeights), std::back_inserter(misWeightPointers),
                       [](const auto& str) { return str.data(); });
        ImGui::Combo("MIS weights (R-MIS)", (int*) &config.features.misWeightRMIS, misWeightPointers.data(), static_cast<int>(misWeightPointers.size()));
        ImGui::Checkbox("Progressive estimator (R-OMIS)", &config.features.useProgressiveROMIS);
        ImGui::SliderInt("Progressive update modulo (R-OMIS)",  (int*) &config.features.progressiveUpdateMod, 1, config.features.maxIterationsMIS);

        ImGui::Spacing();
        ImGui::Separator();

        // ReSTIR parameters
        ImGui::Text("ReSTIR");
        ImGui::SliderInt("Spatial resampling passes",   (int*) &config.features.spatialResamplingPasses,    1, 5);
        ImGui::SliderInt("Temporal M clamp",            (int*) &config.features.temporalClampM,             1, 40);
    }
}

void UiManager::drawToneMappingControls() {
    if (ImGui::CollapsingHeader("Tone Mapping", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Enable tone mapping",  &config.features.enableToneMapping);
        ImGui::SliderFloat("Gamma",             &config.features.gamma, 0.1f, 3.0f, "%.1f");
        ImGui::DragFloat("Exposure",            &config.features.exposure, 0.0001f, 0.0005f, 1.5f, "%.4f");
    }
}
