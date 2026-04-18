#pragma once

#include <cstddef>
#include <vector>

#include "math/types.hpp"

constexpr float kArenaHalfWidth = 450.0f;
constexpr float kArenaHalfHeight = 300.0f;
constexpr float kArenaWallHeight = 120.0f; // Debug draw only; collision is infinite vertical planes.
constexpr float kTopRadius = 45.0f;
// Nominal spin rate about world +Y (upright top); slingshot launch does not add tilt or off-axis ω.
constexpr float kTopSpinAboutWorldY = 14.0f;
constexpr int kCircleSegments = 64;

struct RigidBody {
    Vec3 position;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Quaternion orientation;
    float mass;
    Vec3 invInertiaBody{};
    float radius = kTopRadius;
};

extern std::vector<RigidBody> g_rigidBodies;

// Only this body is reset / slingshot-launched with Z + floor drag (single “beyblade”).
constexpr std::size_t kLaunchableBodyIndex = 0;

inline RigidBody& launchableTop() {
    return g_rigidBodies[kLaunchableBodyIndex];
}

constexpr float kArenaFloorY = 0.0f;

constexpr Vec3 kGravity = {0.0f, -520.0f, 0.0f};
constexpr float kContactRestitution = 0.68f;
constexpr float kFrictionMu = 0.42f;

constexpr float kFixedTimeStepSeconds = 1.0f / 120.0f;
constexpr int kMaxFixedStepsPerFrame = 8;
extern float g_fixedStepAccumulator;

void integrateRigidBody(RigidBody& body, float dt);
