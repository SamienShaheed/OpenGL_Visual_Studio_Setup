#pragma once

#include <vector>

#include "math/math.hpp"
#include "math/types.hpp"

struct GLFWwindow;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

bool initDebugLineRenderer();
void shutdownDebugLineRenderer();

void drawLineBatch(const std::vector<Vec3>& vertices, unsigned int primitive, const Vec3& color, const Mat4& mvp);
void drawWorldAxesGizmo(const Mat4& view, int fbw, int fbh);

// COM velocity: 3D arrow (speed-scaled), XZ-only slide line in the plane through COM (same height as top ring),
// optional vertical tick; hidden when |v| is tiny.
void drawVelocityGuides(const Vec3& com, const Vec3& linearVelocity, float topRadius, const Mat4& mvp);

void appendCircleXZAtY(std::vector<Vec3>& outVerts, float cx, float cy, float cz, float radius, int segments);
void appendArenaGrid(std::vector<Vec3>& outVerts, float halfW, float halfD, float y, int divisions);
void appendArenaBorder(std::vector<Vec3>& outVerts, float halfW, float halfD, float y);
void appendArenaWallFrame(std::vector<Vec3>& outVerts, float halfW, float halfD, float floorY, float wallHeight);
