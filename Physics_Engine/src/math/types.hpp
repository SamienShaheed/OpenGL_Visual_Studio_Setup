#pragma once

struct Vec2 {
    float x, y;
};

struct Vec3 {
    float x, y, z;
};

struct Mat4 {
    float m[16];
};

struct Quaternion {
    float w, x, y, z;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }
