#include "physics/physics.hpp"

#include "math/math.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

std::vector<RigidBody> g_rigidBodies = {
    {
        {0.0f, kTopRadius, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, kTopSpinAboutWorldY, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f},
        1.0f,
        {0.0f, 0.0f, 0.0f},
        kTopRadius,
    },
    {
        {180.0f, kTopRadius, 80.0f},
        {-90.0f, 0.0f, 55.0f},
        {0.8f, 12.0f, 2.0f},
        {1.0f, 0.0f, 0.0f, 0.0f},
        1.0f,
        {0.0f, 0.0f, 0.0f},
        kTopRadius,
    },
};

float g_fixedStepAccumulator = 0.0f;

// n: unit normal from solid into arena (air). r: COM -> contact on body (world).
static void resolveContactImpulses(
    RigidBody& body,
    const Vec3& n,
    const Vec3& r,
    float dt,
    bool dampGroundTilt,
    bool useGravityFrictionCap) {
    auto applyImpulse = [&](const Vec3& impulse) {
        body.linearVelocity = body.linearVelocity + impulse * (1.0f / body.mass);
        body.angularVelocity =
            body.angularVelocity + applyInvInertiaWorld(body.orientation, cross(r, impulse), body.invInertiaBody);
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
        const float jn = -(1.0f + kContactRestitution) * vn / denom;
        applyImpulse(n * jn);
        jnApplied = std::fabs(jn);
    }

    vRel = body.linearVelocity + cross(body.angularVelocity, r);
    Vec3 vT = vRel - n * dot(vRel, n);
    const float vtLen = length(vT);
    if (vtLen < 1.0e-5f) {
        return;
    }

    const Vec3 t = vT * (-1.0f / vtLen);
    const Vec3 rxt = cross(r, t);
    const Vec3 wInvT = applyInvInertiaWorld(body.orientation, rxt, body.invInertiaBody);
    const Vec3 kT = cross(wInvT, r);
    float denomT = (1.0f / body.mass) + dot(t, kT);
    if (denomT < 1.0e-8f) {
        return;
    }

    const float jtFree = -dot(vRel, t) / denomT;

    const float gravitySupportImpulse = body.mass * length(kGravity) * dt * 0.45f;
    const float jMax = kFrictionMu * (useGravityFrictionCap ? std::max(jnApplied, gravitySupportImpulse) : jnApplied);

    const float jt = std::clamp(jtFree, -jMax, jMax);
    applyImpulse(t * jt);

    if (dampGroundTilt) {
        body.angularVelocity.x *= 0.985f;
        body.angularVelocity.z *= 0.985f;
    }
}

static void resolvePlaneContact(RigidBody& body, float planeY, float ballRadius, float dt) {
    const Vec3 n = {0.0f, 1.0f, 0.0f};
    const float bottomY = body.position.y - ballRadius;
    if (bottomY > planeY) {
        return;
    }

    const float penetration = planeY - bottomY;
    body.position = body.position + n * penetration;

    const Vec3 r = n * (-ballRadius);
    resolveContactImpulses(body, n, r, dt, true, true);
}

static void resolveArenaWalls(RigidBody& body, float ballRadius, float halfW, float halfD, float dt) {
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
        resolveContactImpulses(body, n, r, dt, false, false);
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
        resolveContactImpulses(body, n, r, dt, false, false);
    };

    wallX(true);
    wallX(false);
    wallZ(true);
    wallZ(false);
}

void integrateRigidBody(RigidBody& body, float dt) {
    body.linearVelocity = body.linearVelocity + kGravity * dt;
    body.position = body.position + body.linearVelocity * dt;

    resolvePlaneContact(body, kArenaFloorY, body.radius, dt);
    resolveArenaWalls(body, body.radius, kArenaHalfWidth, kArenaHalfHeight, dt);

    const float angularSpeed = length(body.angularVelocity);
    if (angularSpeed > 0.00001f) {
        const Quaternion deltaRotation = quatFromAxisAngle(body.angularVelocity, angularSpeed * dt);
        body.orientation = quatNormalize(quatMultiply(deltaRotation, body.orientation));
    }
}
