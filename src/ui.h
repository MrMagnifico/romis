#pragma once
#ifndef _UI_H_
#define _UI_H_

#include "bvh_interface.h"
#include "common.h"
#include "config.h"
#include "reservoir.h"
#include "scene.h"
#include "screen.h"

#include <framework/trackball.h>


class UiManager {
public:
    UiManager(BvhInterface& bvh, Trackball& camera, Config& config, std::optional<Ray>& optDebugRay,
              std::shared_ptr<const ReservoirGrid>& previousFrameGrid, Scene& scene, SceneType& sceneType,
              Screen& screen, ViewMode& viewMode, Window& window,
              int& bvhDebugLevel, int& bvhDebugLeaf, bool& debugBVHLevel, bool& debugBVHLeaf, int& selectedLightIdx);

    void draw();

private:
    // External object handles
    BvhInterface& bvh;
    Trackball& camera;
    Config& config;
    std::optional<Ray>& optDebugRay;
    std::shared_ptr<const ReservoirGrid>& previousFrameGrid;
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
    void drawExtraFeaturesToggles();
    void drawCameraStats();
    void drawRenderToFile();
    void drawBvhDebug();
    void drawLightControls();

    // ReSTIR UI bits
    void drawRestirTab();
    void drawRestirFeaturesToggles();
    void drawRestirParams();

    // Misc UI bits
    void drawMiscTab();
    void drawToneMappingControls();
};

#endif
