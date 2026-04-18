#pragma once

#include "math/types.hpp"

// Runtime simulation & gameplay tuning (edited via ImGui). Defaults match prior constexpr values.
struct SimulationTuning {
    Vec3 gravity{0.0f, -520.0f, 0.0f};

    float contactRestitution = 0.68f;
    float frictionMuFloor = 0.42f;
    float sphereSphereRestitution = 0.88f;
    float sphereSphereFrictionMu = 0.22f;

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

    int sphereSphereIterations = 6;

    // B: spin / translation decay (per second; 0 = off).
    float angularDampingPerSecond = 0.10f;
    float linearAirDragPerSecond = 0.04f;
    float linearVerticalAirDragPerSecond = 0.04f;
    // Multiplier on angular damping about the body spin axis (lower = keeps spin longer vs wobble).
    float angularDampingSpinAxisScale = 0.38f;
    // Extra decay on ω perpendicular to body spin axis (tumble); 0 = off.
    float angularTumbleDampingPerSecond = 0.20f;
    // Extra tumble decay when lowest collider is near the floor (tip-down recovery); 0 = off.
    float groundedTumbleDampingPerSecond = 0.14f;

    // A: X key random match — equal closing speed, random spins.
    float matchInitialClosingSpeed = 340.0f;
    float matchSpinAbsMax = 36.0f;
    float matchSpinTiltMax = 5.0f;
    float matchArenaInset = 14.0f;
    float matchMinSeparationExtra = 35.0f;
    // After X: skip horizontal air drag for N substeps; scale floor/wall tangential friction (so tops reach each other).
    int matchApproachGraceSubsteps = 260;
    float matchApproachFrictionScale = 0.38f;

    // C: extra tangential coupling when sliding fast + arcade “seek” along XZ.
    float sphereSphereSlipRef = 48.0f;
    float sphereSphereSlipBoost = 0.45f;
    float sphereSphereSlipBoostMax = 2.5f;
    float sphereSphereSeekAccel = 12.0f;
    // Scales normal + tangential + seek impulses (1 = full). <1 tames multi-contact blow-ups.
    float sphereSphereImpulseResponseScale = 0.88f;
    // If true, collision impulses only change angular velocity along world +Y (spin in the horizontal plane).
    // Linear velocity from contacts is unchanged (tops can still pop upward).
    bool collisionAngularDeltaWorldYOnly = true;

    // E: stability / camera — applied each substep after integration (0 = no cap).
    float maxLinearSpeed = 780.0f;
    float maxAngularSpeed = 125.0f;
    // Extra cap on upward Y only (after tip/attack hits); 0 = unused.
    float maxUpwardLinearSpeed = 280.0f;

    // Grounded: lowest collider sphere bottom within this of arenaFloorY counts as on floor.
    float floorContactMargin = 24.0f;

    // F: gyro upright — torque aligns body +Y with world +Y; fades out when spin about axis drops.
    float gyroUprightStrength = 420.0f;
    float gyroUprightSpinDead = 3.5f;
    float gyroUprightSpinFull = 24.0f;
    float gyroUprightMaxTorque = 5600.0f;
    // Multiplies upright torque when grounded (1 = no boost).
    float gyroFloorBoost = 1.55f;

    // H: battle band — XZ pull between both tops while each still has spin; 0 max impulse = off.
    float battleBandMaxImpulsePerStep = 110.0f;
    float battleBandRestDistance = 125.0f;
    float battleBandStiffness = 52.0f;
    float battleBandMinSpinAboutAxis = 4.5f;
    bool battleBandRequireFloorContact = true;

    float body0Mass = 1.0f;
    float body1Mass = 1.0f;
    float body0Radius = 45.0f;
    float body1Radius = 45.0f;

    // D: Beyblade-style compound (sphere soup). Body +Y is the spin axis. Tip below COM; flat disk ring above COM.
    float body0TipRadius = 9.0f;
    float body0TipOffsetBelowCom = 40.0f;
    float body0HubColliderRadius = 20.0f;
    bool body0UseHubCollider = true;
    float body0DiskRingY = 22.0f;
    float body0DiskRadial = 38.0f;
    float body0DiskSphereRadius = 11.0f;
    int body0DiskSphereCount = 6;
    int body0MidAttackCount = 0;
    float body0MidAttackRadius = 12.0f;
    float body0MidAttackRadial = 34.0f;
    float body0MidAttackY = 2.0f;

    float body1TipRadius = 8.0f;
    float body1TipOffsetBelowCom = 38.0f;
    float body1HubColliderRadius = 18.0f;
    bool body1UseHubCollider = true;
    float body1DiskRingY = 20.0f;
    float body1DiskRadial = 36.0f;
    float body1DiskSphereRadius = 10.0f;
    int body1DiskSphereCount = 6;
    int body1MidAttackCount = 0;
    float body1MidAttackRadius = 11.0f;
    float body1MidAttackRadial = 33.0f;
    float body1MidAttackY = 2.0f;
};

extern SimulationTuning g_simTuning;
