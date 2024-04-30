#pragma once
#ifndef _UI_H_
#define _UI_H_

#include <ray_tracing/bvh_interface.h>
#include <rendering/reservoir.h>
#include <rendering/screen.h>
#include <scene/scene.h>
#include <utils/common.h>
#include <utils/config.h>

#include <framework/trackball.h>


class UiManager {
public:
    UiManager(BvhInterface& bvh, Trackball& camera, Config& config, std::optional<Ray>& optDebugRay,
              std::shared_ptr<ReservoirGrid>& previousFrameGrid, Scene& scene, SceneType& sceneType,
              Screen& screen, ViewMode& viewMode, Window& window,
              int& bvhDebugLevel, int& bvhDebugLeaf, bool& debugBVHLevel, bool& debugBVHLeaf, int& selectedLightIdx);

    void draw();

private:
    // External object handles
    BvhInterface& bvh;
    Trackball& camera;
    Config& config;
    std::optional<Ray>& optDebugRay;
    std::shared_ptr<ReservoirGrid>& previousFrameGrid;
    Scene& scene;
    SceneType& sceneType;
    Screen& screen;
    ViewMode& viewMode;
    Window& window;

    // External primitive handles
    int& bvhDebugLevel;
    int& bvhDebugLeaf;
    bool& debugBVHLevel;
    bool& debugBVHLeaf;
    int& selectedLightIdx;

    // Final project template UI bits
    void drawProjectTab();
    void drawSceneSelection();
    void drawViewModeSelection();
    void drawFeaturesToggles();
    void drawCameraStats();
    void drawRenderToFile();
    void drawBvhDebug();
    void drawLightControls();

    // Ray-tracing UI bits
    void drawRayTracingTab();
    void drawRayTracingFeaturesToggles();
    void drawRayTracingParams();

    // Misc UI bits
    void drawMiscTab();
    void drawToneMappingControls();
};

#endif
