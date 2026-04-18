#pragma once

#include "math/types.hpp"

constexpr float kArenaHalfWidth = 450.0f;
constexpr float kArenaHalfHeight = 300.0f;
constexpr float kTopRadius = 45.0f;
constexpr int kCircleSegments = 64;

struct RigidBody {
    Vec3 position;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Quaternion orientation;
    float mass;
    Vec3 invInertiaBody{};
};

extern RigidBody g_top;

constexpr float kArenaFloorY = 0.0f;

constexpr Vec3 kGravity = {0.0f, -520.0f, 0.0f};
constexpr float kContactRestitution = 0.68f;
constexpr float kFrictionMu = 0.42f;

constexpr float kFixedTimeStepSeconds = 1.0f / 120.0f;
constexpr int kMaxFixedStepsPerFrame = 8;
extern float g_fixedStepAccumulator;

void integrateRigidBody(RigidBody& body, float dt);
