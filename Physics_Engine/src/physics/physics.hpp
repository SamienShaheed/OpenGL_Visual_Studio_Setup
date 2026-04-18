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
// Player-adjustable launch spin (Z-drag + [ ]), signed about world +Y; clamped to ±max (rad/s).
// Typical magnitude floor when tuning (not enforced by clamp; brackets can cross zero).
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

// Per rendered frame: accumulated contact/friction stats for stick vs slide debugging (HUD + optional world draw).
struct StickSlideFrameDebug {
    void reset() {
        *this = StickSlideFrameDebug{};
    }

    // Sphere–sphere (counts include every Gauss–Seidel iteration per substep).
    int ssPairInRange = 0;
    int ssTangentialApplied = 0;
    int ssVtBelowThreshold = 0;
    int ssDenomTBad = 0;
    int ssFrictionSaturated = 0;
    int ssFrictionNotSaturated = 0;

    bool ssLastValid = false;
    int ssLastBodyI = 0;
    int ssLastBodyJ = 0;
    Vec3 ssLastContact{};
    Vec3 ssLastN{};
    Vec3 ssLastT{};
    float ssLastJn = 0.0f;
    float ssLastJt = 0.0f;
    float ssLastJtFree = 0.0f;
    float ssLastJMax = 0.0f;
    float ssLastVtLen = 0.0f;
    float ssLastRestingProxy = 0.0f;
    bool ssLastSaturated = false;

    // Floor (upward normal).
    int floorContacts = 0;
    int floorTangentialApplied = 0;
    int floorVtSkip = 0;
    int floorDenomTBad = 0;
    int floorFrictionSaturated = 0;
    int floorFrictionNotSaturated = 0;

    bool floorLastValid = false;
    int floorLastBody = -1;
    Vec3 floorLastContact{};
    Vec3 floorLastN{};
    Vec3 floorLastT{};
    float floorLastJnApplied = 0.0f;
    float floorLastJtFree = 0.0f;
    float floorLastJMax = 0.0f;
    float floorLastJt = 0.0f;
    float floorLastVtLen = 0.0f;
    bool floorLastSaturated = false;

    // Arena walls (vertical normals).
    int wallContacts = 0;
    int wallTangentialApplied = 0;
    int wallVtSkip = 0;
    int wallDenomTBad = 0;
    int wallFrictionSaturated = 0;
    int wallFrictionNotSaturated = 0;

    bool wallLastValid = false;
    int wallLastBody = -1;
    Vec3 wallLastContact{};
    Vec3 wallLastN{};
    Vec3 wallLastT{};
    float wallLastJnApplied = 0.0f;
    float wallLastJtFree = 0.0f;
    float wallLastJMax = 0.0f;
    float wallLastJt = 0.0f;
    float wallLastVtLen = 0.0f;
    bool wallLastSaturated = false;
};

extern StickSlideFrameDebug g_stickSlideFrameDebug;
void resetStickSlideFrameDebug();

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
