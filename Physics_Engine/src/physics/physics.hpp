#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "math/types.hpp"
#include "physics/tuning.hpp"

constexpr int kCircleSegments = 64;

enum class CollisionSphereRole : std::uint8_t {
    Tip = 0,      // performance tip below COM
    Hub = 1,      // central stem / mass near spin axis
    DiskRing = 2, // top “energy layer” ring (flat ring of spheres)
    MidAttack = 3 // optional mid-height bumps
};

// Child sphere in body space (spins with the top). Role is used for debug colors / struts.
struct CollisionSphere {
    Vec3 offsetBody{};
    float radius = 1.0f;
    CollisionSphereRole role = CollisionSphereRole::Hub;
};

constexpr int kMaxCollisionSpheresPerBody = 16;

struct RigidBody {
    Vec3 position;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Quaternion orientation;
    float mass;
    Vec3 invInertiaBody{};
    // Core radius: inertia, tuning sync, and default core collider radius.
    float radius = 45.0f;
    // max_i(|offsetBody_i| + radius_i) — spawn / match separation; updated in rebuildRigidBodyCompoundColliders.
    float colliderBoundingRadius = 45.0f;
    std::array<CollisionSphere, kMaxCollisionSpheresPerBody> collisionSpheres{
        CollisionSphere{Vec3{0.0f, 0.0f, 0.0f}, 45.0f, CollisionSphereRole::Hub}};
    std::uint8_t collisionSphereCount = 1;
};

extern std::vector<RigidBody> g_rigidBodies;

// Only this body is reset / slingshot-launched with Z + floor drag (single “beyblade”).
constexpr std::size_t kLaunchableBodyIndex = 0;

inline RigidBody& launchableTop() {
    return g_rigidBodies[kLaunchableBodyIndex];
}

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

void recomputeBeybladeInertia(RigidBody& body);
void rebuildRigidBodyCompoundColliders(RigidBody& body, bool isBody0);
void syncRigidBodiesFromTuning();
void resetSimulationTuningToDefaults();

// Randomize both tops in arena: XZ positions, random spins, equal closing speed toward each other.
void randomizeTopsForMatch();

struct LaunchIntent {
    // Horizontal velocity after launch (world XZ; y ignored).
    Vec3 horizontalVelocity{};
    float spinAboutWorldY = 14.0f;
};

// Linear impulse at COM: J [kg·m/s] → Δv = J / m.
void applyLinearImpulseAtCom(RigidBody& body, const Vec3& impulse);

// Angular impulse about world +Y (body upright); Hy is ∫τ_y dt [kg·m²/s].
void applyAngularImpulseAboutWorldYUpright(RigidBody& body, float Hy);

// Upright pose, zero ω/linear, then apply impulses so v and ω_y match intent (Z-launch release).
void applyLaunchIntentFromZSlingshot(RigidBody& body, const LaunchIntent& intent);

// Translation (gravity, floor, walls) → sphere–sphere pairs → orientation integration.
void physicsFixedSubstep(float dt, bool skipLaunchableBody);
