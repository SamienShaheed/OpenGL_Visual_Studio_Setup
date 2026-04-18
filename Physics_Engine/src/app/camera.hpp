#pragma once

#include "math/types.hpp"

struct GLFWwindow;

constexpr float kIsometricPitchRadians = 0.6154797f;

extern float g_cameraYaw;
extern float g_cameraPitch;
extern float g_cameraDistance;
extern Vec3 g_cameraTarget;

void processInput(GLFWwindow* window, float dt);
void updateCameraMouseOrbit(GLFWwindow* window);
void mouseScrollCallback(GLFWwindow* window, double xOffset, double yOffset);
