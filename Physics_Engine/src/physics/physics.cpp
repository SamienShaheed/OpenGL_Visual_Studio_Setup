#include "physics/physics.hpp"

#include "math/math.hpp"

#include <algorithm>
#include <cmath>

RigidBody g_top = {
    {0.0f, kTopRadius, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 14.0f, 1.5f},
    {1.0f, 0.0f, 0.0f, 0.0f},
    1.0f,
    {0.0f, 0.0f, 0.0f}
};

float g_fixedStepAccumulator = 0.0f;

static void resolvePlaneContact(RigidBody& body, float planeY, float ballRadius, float dt) {
    const Vec3 n = {0.0f, 1.0f, 0.0f};
    const float bottomY = body.position.y - ballRadius;
    if (bottomY > planeY) {
        return;
    }

    const float penetration = planeY - bottomY;
    body.position = body.position + n * penetration;

    const Vec3 r = n * (-ballRadius);

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
    const float jMax = kFrictionMu * std::max(jnApplied, gravitySupportImpulse);

    const float jt = std::clamp(jtFree, -jMax, jMax);
    applyImpulse(t * jt);

    body.angularVelocity.x *= 0.985f;
    body.angularVelocity.z *= 0.985f;
}

void integrateRigidBody(RigidBody& body, float dt) {
    body.linearVelocity = body.linearVelocity + kGravity * dt;
    body.position = body.position + body.linearVelocity * dt;

    resolvePlaneContact(body, kArenaFloorY, kTopRadius, dt);

    const float angularSpeed = length(body.angularVelocity);
    if (angularSpeed > 0.00001f) {
        const Quaternion deltaRotation = quatFromAxisAngle(body.angularVelocity, angularSpeed * dt);
        body.orientation = quatNormalize(quatMultiply(deltaRotation, body.orientation));
    }
}
