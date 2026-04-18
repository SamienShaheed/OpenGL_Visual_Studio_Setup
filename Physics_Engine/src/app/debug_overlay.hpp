#pragma once

struct GLFWwindow;
struct Mat4;

void initDebugOverlay(GLFWwindow* window);
void shutdownDebugOverlay();

void debugOverlayNewFrame();

void debugOverlayDrawHud(
    float frameDt,
    int physicsSubstepsThisFrame,
    float fixedStepAccumulator,
    bool draggingZLaunch,
    float launchSpinY);

void debugOverlayDrawTuningPanel();

// Unity-style world axis trihedron (screen-space, no toolbar). Call each frame before RenderDrawData.
void debugOverlayDrawOrientationGizmo(const Mat4& view, int fbw, int fbh);

void debugOverlayRenderDrawData();

bool debugOverlayWantsMouseCapture();
bool debugOverlayWantsKeyboardCapture();
