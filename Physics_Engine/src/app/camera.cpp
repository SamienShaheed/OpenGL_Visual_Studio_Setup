#include "app/camera.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

#include "math/math.hpp"
#include "math/types.hpp"

float g_cameraYaw = 0.7853982f;
float g_cameraPitch = kIsometricPitchRadians;
float g_cameraDistance = 900.0f;
Vec3 g_cameraTarget = {0.0f, 0.0f, 0.0f};
bool g_isRotatingCamera = false;
double g_lastMouseX = 0.0;
double g_lastMouseY = 0.0;

void processInput(GLFWwindow* window, float dt) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    const float orbitSpeed = 1.8f;
    const float zoomSpeed = 500.0f;

    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        g_cameraYaw -= orbitSpeed * dt;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        g_cameraYaw += orbitSpeed * dt;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        g_cameraDistance += zoomSpeed * dt;
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        g_cameraDistance -= zoomSpeed * dt;
    }

    g_cameraPitch = kIsometricPitchRadians;
    g_cameraDistance = std::clamp(g_cameraDistance, 300.0f, 1800.0f);
}

void updateCameraMouseOrbit(GLFWwindow* window) {
    const int leftPressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    double mouseX = 0.0;
    double mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (leftPressed == GLFW_PRESS) {
        if (!g_isRotatingCamera) {
            g_isRotatingCamera = true;
            g_lastMouseX = mouseX;
            g_lastMouseY = mouseY;
            return;
        }

        const double dx = mouseX - g_lastMouseX;
        const float sensitivity = 0.0055f;

        g_cameraYaw -= static_cast<float>(dx) * sensitivity;
    } else {
        g_isRotatingCamera = false;
    }

    g_lastMouseX = mouseX;
    g_lastMouseY = mouseY;
}

void mouseScrollCallback(GLFWwindow* /*window*/, double /*xOffset*/, double yOffset) {
    g_cameraDistance -= static_cast<float>(yOffset) * 40.0f;
    g_cameraDistance = std::clamp(g_cameraDistance, 300.0f, 1800.0f);
}
