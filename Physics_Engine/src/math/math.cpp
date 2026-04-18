#include "math/math.hpp"

#include <cmath>

float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float length(const Vec3& v) { return std::sqrt(dot(v, v)); }

Vec3 normalize(const Vec3& v) {
    const float len = length(v);
    if (len <= 0.00001f) {
        return {0.0f, 0.0f, 0.0f};
    }
    return v * (1.0f / len);
}

Quaternion quatNormalize(const Quaternion& q) {
    const float len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len <= 0.000001f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float invLen = 1.0f / len;
    return {q.w * invLen, q.x * invLen, q.y * invLen, q.z * invLen};
}

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

Quaternion quatFromAxisAngle(Vec3 axis, float angleRadians) {
    axis = normalize(axis);
    const float half = angleRadians * 0.5f;
    const float s = std::sin(half);
    return quatNormalize({std::cos(half), axis.x * s, axis.y * s, axis.z * s});
}

Vec3 quatRotateVector(const Quaternion& q, const Vec3& v) {
    const Quaternion p = {0.0f, v.x, v.y, v.z};
    const Quaternion qConjugate = {q.w, -q.x, -q.y, -q.z};
    const Quaternion rotated = quatMultiply(quatMultiply(q, p), qConjugate);
    return {rotated.x, rotated.y, rotated.z};
}

Vec3 quatRotateVectorInverse(const Quaternion& q, const Vec3& v) {
    const Quaternion p = {0.0f, v.x, v.y, v.z};
    const Quaternion qConjugate = {q.w, -q.x, -q.y, -q.z};
    const Quaternion rotated = quatMultiply(quatMultiply(qConjugate, p), q);
    return {rotated.x, rotated.y, rotated.z};
}

Vec3 applyInvInertiaWorld(const Quaternion& q, const Vec3& worldVec, const Vec3& invInertiaBody) {
    Vec3 local = quatRotateVectorInverse(q, worldVec);
    local = {local.x * invInertiaBody.x, local.y * invInertiaBody.y, local.z * invInertiaBody.z};
    return quatRotateVector(q, local);
}

Mat4 mat4Identity() {
    return {{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    }};
}

Mat4 mat4Multiply(const Mat4& a, const Mat4& b) {
    Mat4 out = {};
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            out.m[col * 4 + row] =
                a.m[0 * 4 + row] * b.m[col * 4 + 0] +
                a.m[1 * 4 + row] * b.m[col * 4 + 1] +
                a.m[2 * 4 + row] * b.m[col * 4 + 2] +
                a.m[3 * 4 + row] * b.m[col * 4 + 3];
        }
    }
    return out;
}

Mat4 mat4Perspective(float fovYRadians, float aspect, float zNear, float zFar) {
    Mat4 out = {};
    const float f = 1.0f / std::tan(fovYRadians * 0.5f);
    out.m[0] = f / aspect;
    out.m[5] = f;
    out.m[10] = (zFar + zNear) / (zNear - zFar);
    out.m[11] = -1.0f;
    out.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
    return out;
}

Mat4 mat4Ortho(float left, float right, float bottom, float top, float nearVal, float farVal) {
    const float rl = right - left;
    const float tb = top - bottom;
    const float fn = farVal - nearVal;
    Mat4 out = {};
    out.m[0] = 2.0f / rl;
    out.m[5] = 2.0f / tb;
    out.m[10] = -2.0f / fn;
    out.m[12] = -(right + left) / rl;
    out.m[13] = -(top + bottom) / tb;
    out.m[14] = -(farVal + nearVal) / fn;
    out.m[15] = 1.0f;
    return out;
}

Vec3 mat4TransformDirection(const Mat4& m, const Vec3& d) {
    return {
        m.m[0] * d.x + m.m[4] * d.y + m.m[8] * d.z,
        m.m[1] * d.x + m.m[5] * d.y + m.m[9] * d.z,
        m.m[2] * d.x + m.m[6] * d.y + m.m[10] * d.z
    };
}

Mat4 mat4LookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalize(center - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);

    Mat4 out = mat4Identity();
    out.m[0] = s.x;
    out.m[1] = u.x;
    out.m[2] = -f.x;
    out.m[4] = s.y;
    out.m[5] = u.y;
    out.m[6] = -f.y;
    out.m[8] = s.z;
    out.m[9] = u.z;
    out.m[10] = -f.z;
    out.m[12] = -dot(s, eye);
    out.m[13] = -dot(u, eye);
    out.m[14] = dot(f, eye);
    return out;
}
