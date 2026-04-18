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
// Player-adjustable launch spin (Z-drag + [ ]), clamped to this range (rad/s).
constexpr float kLaunchSpinYMin = 4.0f;
constexpr float kLaunchSpinYMax = 40.0f;
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
// Sphere–sphere: bouncier and much less “grip” than floor/wall so tops glance off instead of sticking.
constexpr float kSphereSphereRestitution = 0.82f;
constexpr float kSphereSphereFrictionMu = 0.14f;

constexpr float kFixedTimeStepSeconds = 1.0f / 120.0f;
constexpr int kMaxFixedStepsPerFrame = 8;
extern float g_fixedStepAccumulator;

struct LaunchIntent {
    // Horizontal velocity after launch (world XZ; y ignored).
    Vec3 horizontalVelocity{};
    float spinAboutWorldY = kTopSpinAboutWorldY;
};

// Linear impulse at COM: J [kg·m/s] → Δv = J / m.
void applyLinearImpulseAtCom(RigidBody& body, const Vec3& impulse);

// Angular impulse about world +Y (body upright); Hy is ∫τ_y dt [kg·m²/s].
void applyAngularImpulseAboutWorldYUpright(RigidBody& body, float Hy);

// Upright pose, zero ω/linear, then apply impulses so v and ω_y match intent (Z-launch release).
void applyLaunchIntentFromZSlingshot(RigidBody& body, const LaunchIntent& intent);

// Translation (gravity, floor, walls) → sphere–sphere pairs → orientation integration.
void physicsFixedSubstep(float dt, bool skipLaunchableBody);
