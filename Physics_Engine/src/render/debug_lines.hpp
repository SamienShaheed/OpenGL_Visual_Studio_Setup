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

void appendCircleXZAtY(std::vector<Vec3>& outVerts, float cx, float cy, float cz, float radius, int segments);
void appendArenaGrid(std::vector<Vec3>& outVerts, float halfW, float halfD, float y, int divisions);
void appendArenaBorder(std::vector<Vec3>& outVerts, float halfW, float halfD, float y);
