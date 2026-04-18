#include "physics/physics.hpp"

#include "math/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
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
}

void resetSimulationTuningToDefaults() {
    g_simTuning = SimulationTuning{};
}

static void applyAirAndAngularDamping(RigidBody& body, float dt) {
    const float ad = g_simTuning.angularDampingPerSecond;
    if (ad > 0.0f) {
        const float s = std::max(0.0f, 1.0f - ad * dt);
        body.angularVelocity = body.angularVelocity * s;
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
    const float r0 = a.radius;
    const float r1 = b.radius;

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
        body.angularVelocity =
            body.angularVelocity + applyInvInertiaWorld(body.orientation, cross(r, impulse), body.invInertiaBody);
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

static void resolvePlaneContact(RigidBody& body, int bodyIndex, float planeY, float ballRadius, float dt) {
    const Vec3 n = {0.0f, 1.0f, 0.0f};
    const float bottomY = body.position.y - ballRadius;
    if (bottomY > planeY) {
        return;
    }

    const float penetration = planeY - bottomY;
    body.position = body.position + n * penetration;

    const Vec3 r = n * (-ballRadius);
    resolveContactImpulses(body, n, r, dt, true, true, bodyIndex, ContactSurfaceKind::Floor);
}

static void resolveArenaWalls(RigidBody& body, int bodyIndex, float ballRadius, float halfW, float halfD, float dt) {
    const auto wallX = [&](bool positiveSide) {
        const Vec3 n = positiveSide ? Vec3{-1.0f, 0.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
        float penetration = 0.0f;
        if (positiveSide) {
            const float limit = halfW;
            if (body.position.x + ballRadius <= limit) {
                return;
            }
            penetration = body.position.x + ballRadius - limit;
        } else {
            const float limit = -halfW;
            if (body.position.x - ballRadius >= limit) {
                return;
            }
            penetration = limit - (body.position.x - ballRadius);
        }
        body.position = body.position + n * penetration;
        const Vec3 r = n * (-ballRadius);
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    const auto wallZ = [&](bool positiveSide) {
        const Vec3 n = positiveSide ? Vec3{0.0f, 0.0f, -1.0f} : Vec3{0.0f, 0.0f, 1.0f};
        float penetration = 0.0f;
        if (positiveSide) {
            const float limit = halfD;
            if (body.position.z + ballRadius <= limit) {
                return;
            }
            penetration = body.position.z + ballRadius - limit;
        } else {
            const float limit = -halfD;
            if (body.position.z - ballRadius >= limit) {
                return;
            }
            penetration = limit - (body.position.z - ballRadius);
        }
        body.position = body.position + n * penetration;
        const Vec3 r = n * (-ballRadius);
        resolveContactImpulses(body, n, r, dt, false, false, bodyIndex, ContactSurfaceKind::Wall);
    };

    wallX(true);
    wallX(false);
    wallZ(true);
    wallZ(false);
}

static void applyImpulseAtContact(RigidBody& body, const Vec3& r, const Vec3& impulse) {
    body.linearVelocity = body.linearVelocity + impulse * (1.0f / body.mass);
    body.angularVelocity =
        body.angularVelocity + applyInvInertiaWorld(body.orientation, cross(r, impulse), body.invInertiaBody);
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

static void resolveSphereSpherePair(RigidBody& a, RigidBody& b, float dt, int idxA, int idxB) {
    Vec3 delta = b.position - a.position;
    float d = length(delta);
    const float minDist = a.radius + b.radius;

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
        delta = b.position - a.position;
        d = length(delta);
        if (d < 1.0e-8f) {
            n = {1.0f, 0.0f, 0.0f};
        } else {
            n = delta * (1.0f / d);
        }
    }

    const Vec3 rA = n * a.radius;
    const Vec3 rB = n * (-b.radius);

    Vec3 vRel = a.linearVelocity + cross(a.angularVelocity, rA) - b.linearVelocity - cross(b.angularVelocity, rB);
    const float vn = dot(vRel, n);

    float jn = 0.0f;
    if (vn < 0.0f) {
        const float invMassSum = 1.0f / a.mass + 1.0f / b.mass;
        const float denom = std::max(invMassSum, 1.0e-12f);
        jn = -(1.0f + g_simTuning.sphereSphereRestitution) * vn / denom;
        applyImpulseAtContact(a, rA, n * jn);
        applyImpulseAtContact(b, rB, n * (-jn));
    }

    if (g_simTuning.sphereSphereSeekAccel > 0.0f) {
        Vec3 rab = b.position - a.position;
        rab.y = 0.0f;
        const float lr = length(rab);
        if (lr > 1.0e-4f) {
            const Vec3 seek = rab * (1.0f / lr);
            const float jSeek = g_simTuning.sphereSphereSeekAccel * dt;
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
    // Floor uses a large gravity proxy when jn≈0; for two tops that makes tangential friction huge and
    // they “stick” and slide. Use a small resting normal only when there was no collision impulse.
    const float minMass = std::min(a.mass, b.mass);
    const float restingNormalImpulse = minMass * length(g_simTuning.gravity) * dt * 0.06f;
    const float jMaxBase = g_simTuning.sphereSphereFrictionMu * std::max(jnAbs, restingNormalImpulse);
    const float slipNorm =
        vtLen / (g_simTuning.sphereSphereSlipRef + 1.0e-5f);
    const float slipBoost =
        1.0f + g_simTuning.sphereSphereSlipBoost * std::min(slipNorm, g_simTuning.sphereSphereSlipBoostMax);
    const float jMax = jMaxBase * slipBoost;

    const float jt = std::clamp(jtFree, -jMax, jMax);
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
    dbg.ssLastContact = a.position + rA;
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

static void integrateRigidBodyTranslation(RigidBody& body, float dt, int bodyIndex) {
    body.linearVelocity = body.linearVelocity + g_simTuning.gravity * dt;
    body.position = body.position + body.linearVelocity * dt;

    resolvePlaneContact(body, bodyIndex, g_simTuning.arenaFloorY, body.radius, dt);
    resolveArenaWalls(
        body, bodyIndex, body.radius, g_simTuning.arenaHalfWidth, g_simTuning.arenaHalfHeight, dt);
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
                resolveSphereSpherePair(g_rigidBodies[i], g_rigidBodies[j], dt, static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
        if (skipLaunchableBody && i == kLaunchableBodyIndex) {
            continue;
        }
        integrateRigidBodyOrientation(g_rigidBodies[i], dt);
        applyAirAndAngularDamping(g_rigidBodies[i], dt);
    }

    if (s_matchApproachGraceSubsteps > 0) {
        --s_matchApproachGraceSubsteps;
    }
}
