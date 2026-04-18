#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "app/app.hpp"
#include "app/camera.hpp"
#include "math/math.hpp"
#include "physics/physics.hpp"
#include "render/debug_lines.hpp"

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Physics Engine 3D Debug View", nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cout << "Failed to initialize GLAD\n";
        glfwTerminate();
        return -1;
    }

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetScrollCallback(window, mouseScrollCallback);
    int fbw = 0;
    int fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);

    if (!initDebugLineRenderer()) {
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.5f);

    initFrameTimer();

    {
        const float m = g_top.mass;
        const float r = kTopRadius;
        const float Ispin = 0.55f * m * r * r;
        const float Itrans = 0.32f * m * r * r;
        g_top.invInertiaBody = {
            1.0f / std::max(Itrans, 1.0e-6f),
            1.0f / std::max(Ispin, 1.0e-6f),
            1.0f / std::max(Itrans, 1.0e-6f)
        };
    }

    std::vector<Vec3> arenaGridVerts;
    arenaGridVerts.reserve(5000);
    appendArenaGrid(arenaGridVerts, kArenaHalfWidth, kArenaHalfHeight, kArenaFloorY, 36);
    appendArenaBorder(arenaGridVerts, kArenaHalfWidth, kArenaHalfHeight, kArenaFloorY);

    while (!glfwWindowShouldClose(window)) {
        const float frameDt = computeDeltaTimeSeconds();
        processInput(window, frameDt);
        updateCameraMouseOrbit(window);
        g_cameraPitch = kIsometricPitchRadians;
        g_fixedStepAccumulator += frameDt;

        int simulatedSteps = 0;
        while (g_fixedStepAccumulator >= kFixedTimeStepSeconds && simulatedSteps < kMaxFixedStepsPerFrame) {
            integrateRigidBody(g_top, kFixedTimeStepSeconds);
            g_fixedStepAccumulator -= kFixedTimeStepSeconds;
            simulatedSteps++;
        }
        if (simulatedSteps == kMaxFixedStepsPerFrame && g_fixedStepAccumulator >= kFixedTimeStepSeconds) {
            g_fixedStepAccumulator = 0.0f;
        }

        glfwGetFramebufferSize(window, &fbw, &fbh);
        const float aspect = (fbh > 0) ? (static_cast<float>(fbw) / static_cast<float>(fbh)) : 1.0f;
        const Mat4 proj = mat4Perspective(1.0472f, aspect, 0.1f, 5000.0f);

        const Vec3 cameraPos = {
            g_cameraTarget.x + std::cos(g_cameraYaw) * std::cos(g_cameraPitch) * g_cameraDistance,
            g_cameraTarget.y + std::sin(g_cameraPitch) * g_cameraDistance,
            g_cameraTarget.z + std::sin(g_cameraYaw) * std::cos(g_cameraPitch) * g_cameraDistance
        };
        const Mat4 view = mat4LookAt(cameraPos, g_cameraTarget, {0.0f, 1.0f, 0.0f});
        const Mat4 mvp = mat4Multiply(proj, view);

        glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawLineBatch(arenaGridVerts, GL_LINES, {0.35f, 0.42f, 0.52f}, mvp);

        std::vector<Vec3> marker;
        marker.reserve(kCircleSegments);
        appendCircleXZAtY(
            marker, g_top.position.x, g_top.position.y, g_top.position.z, kTopRadius, kCircleSegments);
        drawLineBatch(marker, GL_LINE_LOOP, {1.0f, 0.9f, 0.25f}, mvp);

        const Vec3 markerCenter = g_top.position;
        const Vec3 velDir = normalize(g_top.linearVelocity);
        std::vector<Vec3> velLine = {markerCenter, markerCenter + velDir * (kTopRadius * 2.2f)};
        drawLineBatch(velLine, GL_LINES, {1.0f, 0.5f, 0.1f}, mvp);

        const Vec3 localX = quatRotateVector(g_top.orientation, {1.0f, 0.0f, 0.0f});
        const Vec3 localY = quatRotateVector(g_top.orientation, {0.0f, 1.0f, 0.0f});
        const Vec3 localZ = quatRotateVector(g_top.orientation, {0.0f, 0.0f, 1.0f});
        const float axisLen = kTopRadius * 1.8f;
        std::vector<Vec3> topAxisX = {markerCenter, markerCenter + localX * axisLen};
        std::vector<Vec3> topAxisY = {markerCenter, markerCenter + localY * axisLen};
        std::vector<Vec3> topAxisZ = {markerCenter, markerCenter + localZ * axisLen};
        drawLineBatch(topAxisX, GL_LINES, {0.95f, 0.35f, 0.35f}, mvp);
        drawLineBatch(topAxisY, GL_LINES, {0.35f, 0.95f, 0.35f}, mvp);
        drawLineBatch(topAxisZ, GL_LINES, {0.35f, 0.65f, 1.0f}, mvp);

        drawWorldAxesGizmo(view, fbw, fbh);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    shutdownDebugLineRenderer();
    glfwTerminate();
    return 0;
}
