#pragma once

#include "math/types.hpp"

// Runtime simulation & gameplay tuning (edited via ImGui). Defaults match prior constexpr values.
struct SimulationTuning {
    Vec3 gravity{0.0f, -520.0f, 0.0f};

    float contactRestitution = 0.68f;
    float frictionMuFloor = 0.42f;
    float sphereSphereRestitution = 0.82f;
    float sphereSphereFrictionMu = 0.14f;

    float arenaHalfWidth = 450.0f;
    float arenaHalfHeight = 300.0f;
    float arenaFloorY = 0.0f;
    float arenaWallHeight = 120.0f;

    float defaultTopSpinY = 14.0f;

    float launchSpinYMax = 40.0f;

    float fixedDt = 1.0f / 120.0f;
    int maxFixedStepsPerFrame = 8;

    float launchDragMin = 0.5f;
    float launchStrength = 2.2f;
    float maxLaunchSpeed = 720.0f;
    float launchSpinAdjustRate = 10.0f;

    float inertiaSpinFactor = 0.55f;
    float inertiaTransFactor = 0.32f;

    int sphereSphereIterations = 4;

    float body0Mass = 1.0f;
    float body1Mass = 1.0f;
    float body0Radius = 45.0f;
    float body1Radius = 45.0f;
};

extern SimulationTuning g_simTuning;
