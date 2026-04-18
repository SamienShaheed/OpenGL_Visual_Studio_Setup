#pragma once

#include "types.hpp"

float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
float length(const Vec3& v);
Vec3 normalize(const Vec3& v);

Quaternion quatNormalize(const Quaternion& q);
Quaternion quatMultiply(const Quaternion& a, const Quaternion& b);
Quaternion quatFromAxisAngle(Vec3 axis, float angleRadians);
Vec3 quatRotateVector(const Quaternion& q, const Vec3& v);
Vec3 quatRotateVectorInverse(const Quaternion& q, const Vec3& v);
Vec3 applyInvInertiaWorld(const Quaternion& q, const Vec3& worldVec, const Vec3& invInertiaBody);

Mat4 mat4Identity();
Mat4 mat4Multiply(const Mat4& a, const Mat4& b);
Mat4 mat4Perspective(float fovYRadians, float aspect, float zNear, float zFar);
Mat4 mat4Ortho(float left, float right, float bottom, float top, float nearVal, float farVal);
Vec3 mat4TransformDirection(const Mat4& m, const Vec3& d);
Mat4 mat4LookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
