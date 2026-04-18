#include "physics/physics.hpp"

#include "math/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

SimulationTuning g_simTuning{};

// After X random match: suppress linear air drag and reduce floor/wall friction until first contact / timeout.
static int s_matchApproachGraceSubsteps = 0;

std::vector<RigidBody> g_rigidBodies = {
    {
        {0.0f, g_simTuning.body0Radius, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, g_simTuning.defaultTopSpinY, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f},
        g_simTuning.body0Mass,
        {0.0f, 0.0f, 0.0f},
        g_simTuning.body0Radius,
    },
    {
        {180.0f, g_simTuning.body1Radius, 80.0f},
        {-90.0f, 0.0f, 55.0f},
        {0.8f, 12.0f, 2.0f},
        {1.0f, 0.0f, 0.0f, 0.0f},
        g_simTuning.body1Mass,
        {0.0f, 0.0f, 0.0f},
        g_simTuning.body1Radius,
    },
};

float g_fixedStepAccumulator = 0.0f;

StickSlideFrameDebug g_stickSlideFrameDebug{};

void resetStickSlideFrameDebug() {
    g_stickSlideFrameDebug.reset();
}

void recomputeBeybladeInertia(RigidBody& body) {
    const float m = body.mass;
    const float r = body.radius;
    const float Ispin = g_simTuning.inertiaSpinFactor * m * r * r;
    const float Itrans = g_simTuning.inertiaTransFactor * m * r * r;
    body.invInertiaBody = {
        1.0f / std::max(Itrans, 1.0e-6f),
        1.0f / std::max(Ispin, 1.0e-6f),
        1.0f / std::max(Itrans, 1.0e-6f)
    };
}

void rebuildRigidBodyCompoundColliders(RigidBody& body, bool isBody0) {
    body.collisionSphereCount = 0;
    auto push = [&](const Vec3& o, float r, CollisionSphereRole role) {
        if (r <= 0.0f || body.collisionSphereCount >= kMaxCollisionSpheresPerBody) {
            return;
        }
        body.collisionSpheres[body.collisionSphereCount++] = {o, r, role};
    };

    const float tipR = isBody0 ? g_simTuning.body0TipRadius : g_simTuning.body1TipRadius;
    const float tipDown =
        std::max(0.0f, isBody0 ? g_simTuning.body0TipOffsetBelowCom : g_simTuning.body1TipOffsetBelowCom);
    const float hubR = isBody0 ? g_simTuning.body0HubColliderRadius : g_simTuning.body1HubColliderRadius;
    const bool useHub = isBody0 ? g_simTuning.body0UseHubCollider : g_simTuning.body1UseHubCollider;
    const float diskY = isBody0 ? g_simTuning.body0DiskRingY : g_simTuning.body1DiskRingY;
    const float diskRadial =
        std::max(0.0f, isBody0 ? g_simTuning.body0DiskRadial : g_simTuning.body1DiskRadial);
    const float diskSR =
        std::max(0.5f, isBody0 ? g_simTuning.body0DiskSphereRadius : g_simTuning.body1DiskSphereRadius);
    int diskCount = isBody0 ? g_simTuning.body0DiskSphereCount : g_simTuning.body1DiskSphereCount;
    diskCount = std::clamp(diskCount, 0, kMaxCollisionSpheresPerBody);

    int midCount = isBody0 ? g_simTuning.body0MidAttackCount : g_simTuning.body1MidAttackCount;
    midCount = std::clamp(midCount, 0, kMaxCollisionSpheresPerBody);
    const float midR =
        std::max(0.5f, isBody0 ? g_simTuning.body0MidAttackRadius : g_simTuning.body1MidAttackRadius);
    const float midRadial =
        std::max(0.0f, isBody0 ? g_simTuning.body0MidAttackRadial : g_simTuning.body1MidAttackRadial);
    const float midY = isBody0 ? g_simTuning.body0MidAttackY : g_simTuning.body1MidAttackY;

    constexpr float kPi = 3.14159265f;

    if (tipR > 0.0f) {
        push({0.0f, -tipDown, 0.0f}, tipR, CollisionSphereRole::Tip);
    }
    if (useHub && hubR > 0.0f) {
        push({0.0f, 0.0f, 0.0f}, hubR, CollisionSphereRole::Hub);
    }
    if (body.collisionSphereCount == 0) {
        push({0.0f, 0.0f, 0.0f}, std::max(body.radius, 4.0f), CollisionSphereRole::Hub);
    }

    int slots = kMaxCollisionSpheresPerBody - static_cast<int>(body.collisionSphereCount);
    const int diskN = std::min(diskCount, std::max(0, slots));
    for (int k = 0; k < diskN; ++k) {
        const float ang = (diskN > 0) ? (2.0f * kPi * static_cast<float>(k) / static_cast<float>(diskN)) : 0.0f;
        const float x = std::cos(ang) * diskRadial;
        const float z = std::sin(ang) * diskRadial;
        push({x, diskY, z}, diskSR, CollisionSphereRole::DiskRing);
    }

    slots = kMaxCollisionSpheresPerBody - static_cast<int>(body.collisionSphereCount);
    const int midN = std::min(midCount, std::max(0, slots));
    for (int k = 0; k < midN; ++k) {
        const float ang = (midN > 0) ? (2.0f * kPi * static_cast<float>(k) / static_cast<float>(midN)) : 0.0f;
        const float x = std::cos(ang) * midRadial;
        const float z = std::sin(ang) * midRadial;
        push({x, midY, z}, midR, CollisionSphereRole::MidAttack);
    }

    float br = 0.0f;
    for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
        const CollisionSphere& s = body.collisionSpheres[i];
        const float ext = length(s.offsetBody) + s.radius;
        br = std::max(br, ext);
    }
    body.colliderBoundingRadius = std::max(br, body.radius * 0.25f);
}

void syncRigidBodiesFromTuning() {
    if (g_rigidBodies.size() < 2) {
        return;
    }
    g_rigidBodies[0].mass = g_simTuning.body0Mass;
    g_rigidBodies[0].radius = g_simTuning.body0Radius;
    g_rigidBodies[1].mass = g_simTuning.body1Mass;
    g_rigidBodies[1].radius = g_simTuning.body1Radius;
    recomputeBeybladeInertia(g_rigidBodies[0]);
    recomputeBeybladeInertia(g_rigidBodies[1]);
    rebuildRigidBodyCompoundColliders(g_rigidBodies[0], true);
    rebuildRigidBodyCompoundColliders(g_rigidBodies[1], false);
}

void resetSimulationTuningToDefaults() {
    g_simTuning = SimulationTuning{};
}

static float rigidBodyLowestColliderBottomY(const RigidBody& body) {
    float minY = 1.0e30f;
    for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
        const CollisionSphere& s = body.collisionSpheres[i];
        const Vec3 wc = body.position + quatRotateVector(body.orientation, s.offsetBody);
        minY = std::min(minY, wc.y - s.radius);
    }
    return minY;
}

static bool rigidBodyContactsFloor(const RigidBody& body, float floorY, float margin) {
    if (body.collisionSphereCount == 0) {
        return body.position.y - body.radius <= floorY + margin;
    }
    return rigidBodyLowestColliderBottomY(body) <= floorY + margin;
}

static void applyCollisionAngularDelta(RigidBody& body, const Vec3& deltaW) {
    if (g_simTuning.collisionAngularDeltaWorldYOnly) {
        body.angularVelocity.y += deltaW.y;
    } else {
        body.angularVelocity = body.angularVelocity + deltaW;
    }
}

static void applyAirAndAngularDamping(RigidBody& body, float dt) {
    const float ad = g_simTuning.angularDampingPerSecond;
    Vec3 spinAxis = quatRotateVector(body.orientation, Vec3{0.0f, 1.0f, 0.0f});
    const float axisLen = length(spinAxis);
    if (axisLen < 1.0e-5f) {
        if (ad > 0.0f) {
            const float s = std::max(0.0f, 1.0f - ad * dt);
            body.angularVelocity = body.angularVelocity * s;
        }
    } else {
        spinAxis = spinAxis * (1.0f / axisLen);
        if (ad > 0.0f) {
            const float yScale = std::clamp(g_simTuning.angularDampingSpinAxisScale, 0.0f, 2.0f);
            const Vec3 w = body.angularVelocity;
            const float wAlong = dot(w, spinAxis);
            const Vec3 wPerp = w - spinAxis * wAlong;
            const float sPerp = std::max(0.0f, 1.0f - ad * dt);
            const float sAlong = std::max(0.0f, 1.0f - ad * yScale * dt);
            body.angularVelocity = spinAxis * (wAlong * sAlong) + wPerp * sPerp;
        }

        const float tumble = g_simTuning.angularTumbleDampingPerSecond;
        if (tumble > 0.0f) {
            Vec3 w2 = body.angularVelocity;
            const float along2 = dot(w2, spinAxis);
            const Vec3 perp2 = w2 - spinAxis * along2;
            const float st = std::max(0.0f, 1.0f - tumble * dt);
            body.angularVelocity = spinAxis * along2 + perp2 * st;
        }
        const float gTumble = g_simTuning.groundedTumbleDampingPerSecond;
        if (gTumble > 0.0f
            && rigidBodyContactsFloor(
                body, g_simTuning.arenaFloorY, g_simTuning.floorContactMargin)) {
            Vec3 w3 = body.angularVelocity;
            const float along3 = dot(w3, spinAxis);
            const Vec3 perp3 = w3 - spinAxis * along3;
            const float sg = std::max(0.0f, 1.0f - gTumble * dt);
            body.angularVelocity = spinAxis * along3 + perp3 * sg;
        }
    }
    const float ld = g_simTuning.linearAirDragPerSecond;
    if (ld > 0.0f && s_matchApproachGraceSubsteps <= 0) {
        const float s = std::max(0.0f, 1.0f - ld * dt);
        body.linearVelocity.x *= s;
        body.linearVelocity.z *= s;
    }
    const float vy = g_simTuning.linearVerticalAirDragPerSecond;
    if (vy > 0.0f && s_matchApproachGraceSubsteps <= 0) {
        body.linearVelocity.y *= std::max(0.0f, 1.0f - vy * dt);
    }
}

void randomizeTopsForMatch() {
    if (g_rigidBodies.size() < 2) {
        return;
    }
    thread_local std::mt19937 rng(std::random_device{}());

    RigidBody& a = g_rigidBodies[0];
    RigidBody& b = g_rigidBodies[1];

    const float halfW = g_simTuning.arenaHalfWidth;
    const float halfH = g_simTuning.arenaHalfHeight;
    const float floorY = g_simTuning.arenaFloorY;
    const float inset = g_simTuning.matchArenaInset;
    const float r0 = a.colliderBoundingRadius;
    const float r1 = b.colliderBoundingRadius;

    const float minXZ = std::max(r0 + inset, 1.0f);
    const float maxX0 = halfW - minXZ;
    const float maxZ0 = halfH - minXZ;
    if (maxX0 <= 0.0f || maxZ0 <= 0.0f) {
        return;
    }

    const float minDistXZ = r0 + r1 + g_simTuning.matchMinSeparationExtra;

    std::uniform_real_distribution<float> ux0(-maxX0, maxX0);
    std::uniform_real_distribution<float> uz0(-maxZ0, maxZ0);
    std::uniform_real_distribution<float> ux1(-maxX0, maxX0);
    std::uniform_real_distribution<float> uz1(-maxZ0, maxZ0);

    float x0 = 0.0f;
    float z0 = 0.0f;
    float x1 = 0.0f;
    float z1 = 0.0f;
    for (int attempt = 0; attempt < 400; ++attempt) {
        x0 = ux0(rng);
        z0 = uz0(rng);
        x1 = ux1(rng);
        z1 = uz1(rng);
        const float dx = x1 - x0;
        const float dz = z1 - z0;
        if (dx * dx + dz * dz >= minDistXZ * minDistXZ) {
            break;
        }
    }
    {
        const float dx = x1 - x0;
        const float dz = z1 - z0;
        if (dx * dx + dz * dz < minDistXZ * minDistXZ) {
            const float sx = std::min(halfW - minXZ, minDistXZ * 0.55f);
            x0 = -sx;
            z0 = 0.0f;
            x1 = sx;
            z1 = 0.0f;
        }
    }

    a.position = {x0, floorY + r0, z0};
    b.position = {x1, floorY + r1, z1};

    float dx = x1 - x0;
    float dz = z1 - z0;
    float lenXZ = std::sqrt(dx * dx + dz * dz);
    if (lenXZ < 1.0e-4f) {
        dx = 1.0f;
        dz = 0.0f;
        lenXZ = 1.0f;
    }
    const Vec3 dir = {dx / lenXZ, 0.0f, dz / lenXZ};
    const float speed = g_simTuning.matchInitialClosingSpeed;
    a.linearVelocity = {dir.x * speed, 0.0f, dir.z * speed};
    b.linearVelocity = {-dir.x * speed, 0.0f, -dir.z * speed};

    std::uniform_real_distribution<float> spinY(-g_simTuning.matchSpinAbsMax, g_simTuning.matchSpinAbsMax);
    std::uniform_real_distribution<float> tilt(-g_simTuning.matchSpinTiltMax, g_simTuning.matchSpinTiltMax);
    a.angularVelocity = {tilt(rng), spinY(rng), tilt(rng)};
    b.angularVelocity = {tilt(rng), spinY(rng), tilt(rng)};

    a.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    b.orientation = {1.0f, 0.0f, 0.0f, 0.0f};

    s_matchApproachGraceSubsteps = g_simTuning.matchApproachGraceSubsteps;
}

static bool frictionSaturated(float jtFree, float jMax) {
    return std::fabs(jtFree) > jMax + 1.0e-6f;
}

void applyLinearImpulseAtCom(RigidBody& body, const Vec3& impulse) {
    body.linearVelocity = body.linearVelocity + impulse * (1.0f / body.mass);
}

void applyAngularImpulseAboutWorldYUpright(RigidBody& body, float Hy) {
    body.angularVelocity.y += Hy * body.invInertiaBody.y;
}

void applyLaunchIntentFromZSlingshot(RigidBody& body, const LaunchIntent& intent) {
    body.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    body.linearVelocity = {0.0f, 0.0f, 0.0f};
    body.angularVelocity = {0.0f, 0.0f, 0.0f};

    const Vec3 horizontal = {intent.horizontalVelocity.x, 0.0f, intent.horizontalVelocity.z};
    applyLinearImpulseAtCom(body, horizontal * body.mass);

    const float Iyy = 1.0f / std::max(body.invInertiaBody.y, 1.0e-6f);
    const float Hy = intent.spinAboutWorldY * Iyy;
    applyAngularImpulseAboutWorldYUpright(body, Hy);
}

enum class ContactSurfaceKind { Floor, Wall };

// n: unit normal from solid into arena (air). r: COM -> contact on body (world).
static void resolveContactImpulses(
    RigidBody& body,
    const Vec3& n,
    const Vec3& r,
    float dt,
    bool dampGroundTilt,
    bool useGravityFrictionCap,
    int bodyIndex,
    ContactSurfaceKind surfaceKind) {
    auto applyImpulse = [&](const Vec3& impulse) {
        body.linearVelocity = body.linearVelocity + impulse * (1.0f / body.mass);
        const Vec3 deltaW =
            applyInvInertiaWorld(body.orientation, cross(r, impulse), body.invInertiaBody);
        applyCollisionAngularDelta(body, deltaW);
    };

    const Vec3 contactPoint = body.position + r;

    auto recordFloorWall = [&](float jnApplied, float vtLen, const Vec3& t, float jtFree, float jMax, float jt,
                                bool vtSkipped, bool denomBad) {
        StickSlideFrameDebug& d = g_stickSlideFrameDebug;
        const bool isFloor = surfaceKind == ContactSurfaceKind::Floor;
        if (isFloor) {
            d.floorContacts++;
            if (denomBad) {
                d.floorDenomTBad++;
            } else if (vtSkipped) {
                d.floorVtSkip++;
            } else {
                d.floorTangentialApplied++;
                if (frictionSaturated(jtFree, jMax)) {
                    d.floorFrictionSaturated++;
                } else {
                    d.floorFrictionNotSaturated++;
                }
            }
            d.floorLastValid = true;
            d.floorLastBody = bodyIndex;
            d.floorLastContact = contactPoint;
            d.floorLastN = n;
            d.floorLastT = denomBad || vtSkipped ? Vec3{} : t;
            d.floorLastJnApplied = jnApplied;
            d.floorLastJtFree = jtFree;
            d.floorLastJMax = jMax;
            d.floorLastJt = jt;
            d.floorLastVtLen = vtLen;
            d.floorLastSaturated = !denomBad && !vtSkipped && frictionSaturated(jtFree, jMax);
        } else {
            d.wallContacts++;
            if (denomBad) {
                d.wallDenomTBad++;
            } else if (vtSkipped) {
                d.wallVtSkip++;
            } else {
                d.wallTangentialApplied++;
                if (frictionSaturated(jtFree, jMax)) {
                    d.wallFrictionSaturated++;
                } else {
                    d.wallFrictionNotSaturated++;
                }
            }
            d.wallLastValid = true;
            d.wallLastBody = bodyIndex;
            d.wallLastContact = contactPoint;
            d.wallLastN = n;
            d.wallLastT = denomBad || vtSkipped ? Vec3{} : t;
            d.wallLastJnApplied = jnApplied;
            d.wallLastJtFree = jtFree;
            d.wallLastJMax = jMax;
            d.wallLastJt = jt;
            d.wallLastVtLen = vtLen;
            d.wallLastSaturated = !denomBad && !vtSkipped && frictionSaturated(jtFree, jMax);
        }
    };

    Vec3 vRel = body.linearVelocity + cross(body.angularVelocity, r);

    float jnApplied = 0.0f;
    const float vn = dot(vRel, n);
    if (vn < 0.0f) {
        const Vec3 rxn = cross(r, n);
        const Vec3 wInv = applyInvInertiaWorld(body.orientation, rxn, body.invInertiaBody);
        const Vec3 k = cross(wInv, r);
        float denom = (1.0f / body.mass) + dot(n, k);
        if (denom < 1.0e-8f) {
            denom = 1.0f / body.mass;
        }
        const float jn = -(1.0f + g_simTuning.contactRestitution) * vn / denom;
        applyImpulse(n * jn);
        jnApplied = std::fabs(jn);
    }

    vRel = body.linearVelocity + cross(body.angularVelocity, r);
    Vec3 vT = vRel - n * dot(vRel, n);
    const float vtLen = length(vT);
    if (vtLen < 1.0e-5f) {
        recordFloorWall(jnApplied, vtLen, {}, 0.0f, 0.0f, 0.0f, true, false);
        return;
    }

    const Vec3 t = vT * (-1.0f / vtLen);
    const Vec3 rxt = cross(r, t);
    const Vec3 wInvT = applyInvInertiaWorld(body.orientation, rxt, body.invInertiaBody);
    const Vec3 kT = cross(wInvT, r);
    float denomT = (1.0f / body.mass) + dot(t, kT);
    if (denomT < 1.0e-8f) {
        recordFloorWall(jnApplied, vtLen, t, 0.0f, 0.0f, 0.0f, false, true);
        return;
    }

    const float jtFree = -dot(vRel, t) / denomT;

    const float gravitySupportImpulse = body.mass * length(g_simTuning.gravity) * dt * 0.45f;
    float jMax = g_simTuning.frictionMuFloor
        * (useGravityFrictionCap ? std::max(jnApplied, gravitySupportImpulse) : jnApplied);
    if (s_matchApproachGraceSubsteps > 0
        && (surfaceKind == ContactSurfaceKind::Floor || surfaceKind == ContactSurfaceKind::Wall)) {
        jMax *= g_simTuning.matchApproachFrictionScale;
    }

    const float jt = std::clamp(jtFree, -jMax, jMax);
    applyImpulse(t * jt);

    recordFloorWall(jnApplied, vtLen, t, jtFree, jMax, jt, false, false);

    if (dampGroundTilt) {
        body.angularVelocity.x *= 0.985f;
        body.angularVelocity.z *= 0.985f;
    }
}

static void resolvePlaneContactCompound(RigidBody& body, int bodyIndex, float planeY, float dt) {
    const Vec3 n = {0.0f, 1.0f, 0.0f};
    float lowestBottom = 1.0e30f;
    std::uint8_t deepest = 0;
    for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
        const CollisionSphere& s = body.collisionSpheres[i];
        const Vec3 wc = body.position + quatRotateVector(body.orientation, s.offsetBody);
        const float bottom = wc.y - s.radius;
        if (bottom < lowestBottom) {
            lowestBottom = bottom;
            deepest = i;
        }
    }
    if (lowestBottom > planeY) {
        return;
    }

    const float penetration = planeY - lowestBottom;
    body.position = body.position + n * penetration;

    const CollisionSphere& s = body.collisionSpheres[deepest];
    const Vec3 wc = body.position + quatRotateVector(body.orientation, s.offsetBody);
    const Vec3 contact = {wc.x, wc.y - s.radius, wc.z};
    const Vec3 r = contact - body.position;
    resolveContactImpulses(body, n, r, dt, true, true, bodyIndex, ContactSurfaceKind::Floor);
}

static void resolveArenaWallsCompound(RigidBody& body, int bodyIndex, float halfW, float halfD, float dt) {
    const auto wallPositiveX = [&] {
        const float limit = halfW;
        float maxRight = -1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            maxRight = std::max(maxRight, wc.x + s.radius);
        }
        if (maxRight <= limit) {
            return;
        }
        const Vec3 n{-1.0f, 0.0f, 0.0f};
        const float penetration = maxRight - limit;
        body.position = body.position + n * penetration;

        std::uint8_t best = 0;
        float bestEx = -1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            const float ex = wc.x + s.radius;
            if (ex > bestEx) {
                bestEx = ex;
                best = i;
            }
        }
        const CollisionSphere& s = body.collisionSpheres[best];
        const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
        const Vec3 wc = body.position + wo;
        const Vec3 contact{limit, wc.y, wc.z};
        const Vec3 r = contact - body.position;
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    const auto wallNegativeX = [&] {
        const float limit = -halfW;
        float minLeft = 1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            minLeft = std::min(minLeft, wc.x - s.radius);
        }
        if (minLeft >= limit) {
            return;
        }
        const Vec3 n{1.0f, 0.0f, 0.0f};
        const float penetration = limit - minLeft;
        body.position = body.position + n * penetration;

        std::uint8_t best = 0;
        float bestLeft = 1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            const float left = wc.x - s.radius;
            if (left < bestLeft) {
                bestLeft = left;
                best = i;
            }
        }
        const CollisionSphere& s = body.collisionSpheres[best];
        const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
        const Vec3 wc = body.position + wo;
        const Vec3 contact{limit, wc.y, wc.z};
        const Vec3 r = contact - body.position;
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    const auto wallPositiveZ = [&] {
        const float limit = halfD;
        float maxFront = -1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            maxFront = std::max(maxFront, wc.z + s.radius);
        }
        if (maxFront <= limit) {
            return;
        }
        const Vec3 n{0.0f, 0.0f, -1.0f};
        const float penetration = maxFront - limit;
        body.position = body.position + n * penetration;

        std::uint8_t best = 0;
        float bestEx = -1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            const float ex = wc.z + s.radius;
            if (ex > bestEx) {
                bestEx = ex;
                best = i;
            }
        }
        const CollisionSphere& s = body.collisionSpheres[best];
        const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
        const Vec3 wc = body.position + wo;
        const Vec3 contact{wc.x, wc.y, limit};
        const Vec3 r = contact - body.position;
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    const auto wallNegativeZ = [&] {
        const float limit = -halfD;
        float minBack = 1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            minBack = std::min(minBack, wc.z - s.radius);
        }
        if (minBack >= limit) {
            return;
        }
        const Vec3 n{0.0f, 0.0f, 1.0f};
        const float penetration = limit - minBack;
        body.position = body.position + n * penetration;

        std::uint8_t best = 0;
        float bestBack = 1.0e30f;
        for (std::uint8_t i = 0; i < body.collisionSphereCount; ++i) {
            const CollisionSphere& s = body.collisionSpheres[i];
            const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
            const Vec3 wc = body.position + wo;
            const float back = wc.z - s.radius;
            if (back < bestBack) {
                bestBack = back;
                best = i;
            }
        }
        const CollisionSphere& s = body.collisionSpheres[best];
        const Vec3 wo = quatRotateVector(body.orientation, s.offsetBody);
        const Vec3 wc = body.position + wo;
        const Vec3 contact{wc.x, wc.y, limit};
        const Vec3 r = contact - body.position;
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    wallPositiveX();
    wallNegativeX();
    wallPositiveZ();
    wallNegativeZ();
}

static void applyImpulseAtContact(RigidBody& body, const Vec3& r, const Vec3& impulse) {
    body.linearVelocity = body.linearVelocity + impulse * (1.0f / body.mass);
    const Vec3 deltaW =
        applyInvInertiaWorld(body.orientation, cross(r, impulse), body.invInertiaBody);
    applyCollisionAngularDelta(body, deltaW);
}

static void applyGyroUprightAssist(RigidBody& body, float dt) {
    const float str = g_simTuning.gyroUprightStrength;
    if (str <= 0.0f || dt <= 0.0f) {
        return;
    }

    Vec3 localY = quatRotateVector(body.orientation, Vec3{0.0f, 1.0f, 0.0f});
    const float axisLen = length(localY);
    if (axisLen < 1.0e-5f) {
        return;
    }
    localY = localY * (1.0f / axisLen);

    const float spin = std::fabs(dot(body.angularVelocity, localY));
    const float dead = g_simTuning.gyroUprightSpinDead;
    const float full = g_simTuning.gyroUprightSpinFull;
    if (spin < dead) {
        return;
    }
    float gate = (spin - dead) / std::max(full - dead, 1.0e-3f);
    gate = std::clamp(gate, 0.0f, 1.0f);

    const Vec3 worldUp{0.0f, 1.0f, 0.0f};
    Vec3 torqueAxis = cross(localY, worldUp);
    const float sinA = length(torqueAxis);
    if (sinA < 1.0e-5f) {
        return;
    }
    torqueAxis = torqueAxis * (1.0f / sinA);
    float torqueMag = str * gate * sinA;
    if (rigidBodyContactsFloor(body, g_simTuning.arenaFloorY, g_simTuning.floorContactMargin)) {
        const float boost = std::clamp(g_simTuning.gyroFloorBoost, 0.0f, 3.0f);
        torqueMag *= boost;
    }
    if (g_simTuning.gyroUprightMaxTorque > 0.0f) {
        torqueMag = std::min(torqueMag, g_simTuning.gyroUprightMaxTorque);
    }
    const Vec3 torque = torqueAxis * torqueMag;

    body.angularVelocity =
        body.angularVelocity
        + applyInvInertiaWorld(body.orientation, torque, body.invInertiaBody) * dt;
}

static void applyBattleBand(float dt, bool skipLaunchableBody) {
    if (g_rigidBodies.size() < 2 || skipLaunchableBody) {
        return;
    }
    const float maxJ = g_simTuning.battleBandMaxImpulsePerStep;
    const float k = g_simTuning.battleBandStiffness;
    const float rest = g_simTuning.battleBandRestDistance;
    const float minSpin = g_simTuning.battleBandMinSpinAboutAxis;
    if (maxJ <= 0.0f || k <= 0.0f) {
        return;
    }

    RigidBody& a = g_rigidBodies[0];
    RigidBody& b = g_rigidBodies[1];

    auto spinAboutAxis = [](const RigidBody& body) -> float {
        Vec3 axis = quatRotateVector(body.orientation, Vec3{0.0f, 1.0f, 0.0f});
        const float len = length(axis);
        if (len < 1.0e-5f) {
            return 0.0f;
        }
        axis = axis * (1.0f / len);
        return std::fabs(dot(body.angularVelocity, axis));
    };

    if (std::min(spinAboutAxis(a), spinAboutAxis(b)) < minSpin) {
        return;
    }

    if (g_simTuning.battleBandRequireFloorContact) {
        const float floorY = g_simTuning.arenaFloorY;
        const float margin = std::max(0.0f, g_simTuning.floorContactMargin);
        if (!rigidBodyContactsFloor(a, floorY, margin) || !rigidBodyContactsFloor(b, floorY, margin)) {
            return;
        }
    }

    Vec3 rab = b.position - a.position;
    rab.y = 0.0f;
    const float dist = length(rab);
    if (dist < 1.0e-4f) {
        return;
    }
    const float excess = dist - rest;
    if (excess <= 0.0f) {
        return;
    }

    const Vec3 dir = rab * (1.0f / dist);
    const float J = std::min(maxJ, k * excess * dt);
    applyLinearImpulseAtCom(a, dir * J);
    applyLinearImpulseAtCom(b, dir * (-J));
}

static void clampRigidBodySpeeds(RigidBody& body) {
    const float upCap = g_simTuning.maxUpwardLinearSpeed;
    if (upCap > 0.0f && body.linearVelocity.y > upCap) {
        body.linearVelocity.y = upCap;
    }
    const float vMax = g_simTuning.maxLinearSpeed;
    if (vMax > 0.0f) {
        const float s = length(body.linearVelocity);
        if (s > vMax) {
            body.linearVelocity = body.linearVelocity * (vMax / s);
        }
    }
    const float wMax = g_simTuning.maxAngularSpeed;
    if (wMax > 0.0f) {
        const float s = length(body.angularVelocity);
        if (s > wMax) {
            body.angularVelocity = body.angularVelocity * (wMax / s);
        }
    }
}

// Treat pairs as separated when centers are farther than this past touching distance.
static constexpr float kSphereSeparatedTolerance = 1.0e-5f;

static void separateSphereCenters(RigidBody& a, RigidBody& b, const Vec3& n, float penetration) {
    if (penetration <= 0.0f) {
        return;
    }
    const float invSum = 1.0f / (a.mass + b.mass);
    a.position = a.position - n * (penetration * b.mass * invSum);
    b.position = b.position + n * (penetration * a.mass * invSum);
}

static void resolveOneWorldSpherePair(
    RigidBody& a,
    RigidBody& b,
    const Vec3& woA,
    const Vec3& woB,
    float ra,
    float rb,
    float dt,
    int idxA,
    int idxB) {
    Vec3 wa = a.position + woA;
    Vec3 wb = b.position + woB;
    Vec3 delta = wb - wa;
    float d = length(delta);
    const float minDist = ra + rb;

    if (d > minDist + kSphereSeparatedTolerance) {
        return;
    }

    g_stickSlideFrameDebug.ssPairInRange++;

    Vec3 n;
    if (d < 1.0e-8f) {
        n = {1.0f, 0.0f, 0.0f};
    } else {
        n = delta * (1.0f / d);
    }

    const float penetration = minDist - d;
    if (penetration > 0.0f) {
        separateSphereCenters(a, b, n, penetration);
        wa = a.position + woA;
        wb = b.position + woB;
        delta = wb - wa;
        d = length(delta);
        if (d < 1.0e-8f) {
            n = {1.0f, 0.0f, 0.0f};
        } else {
            n = delta * (1.0f / d);
        }
    }

    const Vec3 contact = wa + n * ra;
    const Vec3 rA = contact - a.position;
    const Vec3 rB = contact - b.position;

    Vec3 vRel = a.linearVelocity + cross(a.angularVelocity, rA) - b.linearVelocity - cross(b.angularVelocity, rB);
    const float vn = dot(vRel, n);

    const float impulseScale = std::clamp(g_simTuning.sphereSphereImpulseResponseScale, 0.0f, 2.0f);

    float jn = 0.0f;
    if (vn < 0.0f) {
        const float invMassSum = 1.0f / a.mass + 1.0f / b.mass;
        const float denom = std::max(invMassSum, 1.0e-12f);
        jn = -(1.0f + g_simTuning.sphereSphereRestitution) * vn / denom;
        jn *= impulseScale;
        applyImpulseAtContact(a, rA, n * jn);
        applyImpulseAtContact(b, rB, n * (-jn));
    }

    if (g_simTuning.sphereSphereSeekAccel > 0.0f) {
        Vec3 rab = b.position - a.position;
        rab.y = 0.0f;
        const float lr = length(rab);
        if (lr > 1.0e-4f) {
            const Vec3 seek = rab * (1.0f / lr);
            const float jSeek = g_simTuning.sphereSphereSeekAccel * dt * impulseScale;
            applyImpulseAtContact(a, rA, seek * (a.mass * jSeek));
            applyImpulseAtContact(b, rB, seek * (-b.mass * jSeek));
        }
    }

    vRel = a.linearVelocity + cross(a.angularVelocity, rA) - b.linearVelocity - cross(b.angularVelocity, rB);
    Vec3 vT = vRel - n * dot(vRel, n);
    const float vtLen = length(vT);
    if (vtLen < 1.0e-5f) {
        g_stickSlideFrameDebug.ssVtBelowThreshold++;
        return;
    }

    const Vec3 t = vT * (-1.0f / vtLen);

    const Vec3 rxtA = cross(rA, t);
    const Vec3 wInvA = applyInvInertiaWorld(a.orientation, rxtA, a.invInertiaBody);
    const Vec3 kA = cross(wInvA, rA);

    const Vec3 rxtB = cross(rB, t);
    const Vec3 wInvB = applyInvInertiaWorld(b.orientation, rxtB, b.invInertiaBody);
    const Vec3 kB = cross(wInvB, rB);

    float denomT = 1.0f / a.mass + 1.0f / b.mass + dot(t, kA) + dot(t, kB);
    if (denomT < 1.0e-8f) {
        g_stickSlideFrameDebug.ssDenomTBad++;
        return;
    }

    const float jtFree = -dot(vRel, t) / denomT;

    const float jnAbs = std::fabs(jn);
    const float minMass = std::min(a.mass, b.mass);
    const float restingNormalImpulse = minMass * length(g_simTuning.gravity) * dt * 0.06f;
    const float jMaxBase = g_simTuning.sphereSphereFrictionMu * std::max(jnAbs, restingNormalImpulse);
    const float slipNorm = vtLen / (g_simTuning.sphereSphereSlipRef + 1.0e-5f);
    const float slipBoost =
        1.0f + g_simTuning.sphereSphereSlipBoost * std::min(slipNorm, g_simTuning.sphereSphereSlipBoostMax);
    const float jMax = jMaxBase * slipBoost;

    float jt = std::clamp(jtFree, -jMax, jMax);
    jt *= impulseScale;
    applyImpulseAtContact(a, rA, t * jt);
    applyImpulseAtContact(b, rB, t * (-jt));

    StickSlideFrameDebug& dbg = g_stickSlideFrameDebug;
    dbg.ssTangentialApplied++;
    const bool sat = frictionSaturated(jtFree, jMax);
    if (sat) {
        dbg.ssFrictionSaturated++;
    } else {
        dbg.ssFrictionNotSaturated++;
    }
    dbg.ssLastValid = true;
    dbg.ssLastBodyI = idxA;
    dbg.ssLastBodyJ = idxB;
    dbg.ssLastContact = contact;
    dbg.ssLastN = n;
    dbg.ssLastT = t;
    dbg.ssLastJn = jn;
    dbg.ssLastJt = jt;
    dbg.ssLastJtFree = jtFree;
    dbg.ssLastJMax = jMax;
    dbg.ssLastVtLen = vtLen;
    dbg.ssLastRestingProxy = restingNormalImpulse;
    dbg.ssLastSaturated = sat;
}

static void resolveCompoundPair(RigidBody& a, RigidBody& b, float dt, int idxA, int idxB) {
    for (std::uint8_t ia = 0; ia < a.collisionSphereCount; ++ia) {
        for (std::uint8_t ib = 0; ib < b.collisionSphereCount; ++ib) {
            const CollisionSphere& sa = a.collisionSpheres[ia];
            const CollisionSphere& sb = b.collisionSpheres[ib];
            const Vec3 woA = quatRotateVector(a.orientation, sa.offsetBody);
            const Vec3 woB = quatRotateVector(b.orientation, sb.offsetBody);
            resolveOneWorldSpherePair(a, b, woA, woB, sa.radius, sb.radius, dt, idxA, idxB);
        }
    }
}


static void integrateRigidBodyTranslation(RigidBody& body, float dt, int bodyIndex) {
    body.linearVelocity = body.linearVelocity + g_simTuning.gravity * dt;
    body.position = body.position + body.linearVelocity * dt;

    resolvePlaneContactCompound(body, bodyIndex, g_simTuning.arenaFloorY, dt);
    resolveArenaWallsCompound(
        body, bodyIndex, g_simTuning.arenaHalfWidth, g_simTuning.arenaHalfHeight, dt);
}

static void integrateRigidBodyOrientation(RigidBody& body, float dt) {
    const float angularSpeed = length(body.angularVelocity);
    if (angularSpeed > 0.00001f) {
        const Quaternion deltaRotation = quatFromAxisAngle(body.angularVelocity, angularSpeed * dt);
        body.orientation = quatNormalize(quatMultiply(deltaRotation, body.orientation));
    }
}

void physicsFixedSubstep(float dt, bool skipLaunchableBody) {
    for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
        if (skipLaunchableBody && i == kLaunchableBodyIndex) {
            continue;
        }
        integrateRigidBodyTranslation(g_rigidBodies[i], dt, static_cast<int>(i));
    }

    for (int iter = 0; iter < g_simTuning.sphereSphereIterations; ++iter) {
        for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
            for (std::size_t j = i + 1; j < g_rigidBodies.size(); ++j) {
                if (skipLaunchableBody && (i == kLaunchableBodyIndex || j == kLaunchableBodyIndex)) {
                    continue;
                }
                resolveCompoundPair(g_rigidBodies[i], g_rigidBodies[j], dt, static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
        if (skipLaunchableBody && i == kLaunchableBodyIndex) {
            continue;
        }
        integrateRigidBodyOrientation(g_rigidBodies[i], dt);
        applyAirAndAngularDamping(g_rigidBodies[i], dt);
        applyGyroUprightAssist(g_rigidBodies[i], dt);
    }

    applyBattleBand(dt, skipLaunchableBody);

    for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
        if (skipLaunchableBody && i == kLaunchableBodyIndex) {
            continue;
        }
        clampRigidBodySpeeds(g_rigidBodies[i]);
    }

    if (s_matchApproachGraceSubsteps > 0) {
        --s_matchApproachGraceSubsteps;
    }
}
