#pragma once

#include <vector>

#include "math/math.hpp"
#include "math/types.hpp"

struct GLFWwindow;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

bool initDebugLineRenderer();
void shutdownDebugLineRenderer();

void drawLineBatch(const std::vector<Vec3>& vertices, unsigned int primitive, const Vec3& color, const Mat4& mvp);

// COM velocity: 3D arrow (speed-scaled), XZ-only slide line in the plane through COM (same height as top ring),
// optional vertical tick; hidden when |v| is tiny.
void drawVelocityGuides(const Vec3& com, const Vec3& linearVelocity, float topRadius, const Mat4& mvp);

void appendCircleXZAtY(std::vector<Vec3>& outVerts, float cx, float cy, float cz, float radius, int segments);
// Three world-axis great circles per sphere (XZ + XY + YZ) for a clear 3D wireframe ball.
void drawWireframeSphereWorldAxes(const Vec3& center, float radius, int segments, const Vec3& color, const Mat4& mvp);

// Flat ring in the body’s local XZ plane at offset (0, yBody, 0) — reads as the “disk” silhouette.
void drawCircleBodyXZPlane(
    const Vec3& comWorld,
    const Quaternion& bodyOrientation,
    float yBody,
    float ringRadius,
    int segments,
    const Vec3& color,
    const Mat4& mvp);
void appendArenaGrid(std::vector<Vec3>& outVerts, float halfW, float halfD, float y, int divisions);
void appendArenaBorder(std::vector<Vec3>& outVerts, float halfW, float halfD, float y);
void appendArenaWallFrame(std::vector<Vec3>& outVerts, float halfW, float halfD, float floorY, float wallHeight);
// y = y0 + k*r^2 paraboloid wireframe inside r <= maxR, plus vertical rim + top ring (wall verts).
void appendBowlParaboloidWireframe(
    std::vector<Vec3>& outVerts, float y0, float k, float maxR, int meridians, int radialSegments);
void appendBowlRimWallFrame(
    std::vector<Vec3>& outVerts, float y0, float k, float maxR, float wallHeight, int segments);
