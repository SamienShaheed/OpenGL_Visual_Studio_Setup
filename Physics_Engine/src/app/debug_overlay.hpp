#pragma once

struct GLFWwindow;

void initDebugOverlay(GLFWwindow* window);
void shutdownDebugOverlay();

void debugOverlayNewFrame();

void debugOverlayDrawHud(
    float frameDt,
    int physicsSubstepsThisFrame,
    float fixedStepAccumulator,
    bool draggingZLaunch,
    float launchSpinY);

void debugOverlayRenderDrawData();

bool debugOverlayWantsMouseCapture();
