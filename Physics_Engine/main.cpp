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

namespace {

constexpr float kPerspectiveFovY = 1.0472f;

constexpr float kLaunchDragMin = 0.5f;
constexpr float kLaunchStrength = 2.2f;
constexpr float kMaxLaunchSpeed = 720.0f;

bool rayIntersectPlaneY(const Vec3& eye, const Vec3& rayDir, float planeY, Vec3& outHit) {
    if (std::fabs(rayDir.y) < 1.0e-8f) {
        return false;
    }
    const float t = (planeY - eye.y) / rayDir.y;
    if (t < 0.0f) {
        return false;
    }
    outHit = eye + rayDir * t;
    return true;
}

// World-space ray direction through pixel (mx, my) for orbit camera (same construction as proj * view).
Vec3 rayDirFromPixel(
    int fbw,
    int fbh,
    double mx,
    double my,
    const Vec3& eye,
    const Vec3& target) {
    const float aspect = (fbh > 0) ? (static_cast<float>(fbw) / static_cast<float>(fbh)) : 1.0f;
    const float ndcX = (2.0f * static_cast<float>(mx) / static_cast<float>(fbw)) - 1.0f;
    const float ndcY = 1.0f - (2.0f * static_cast<float>(my) / static_cast<float>(fbh));

    const float tanHalf = std::tan(kPerspectiveFovY * 0.5f);
    Vec3 forward = normalize(target - eye);
    Vec3 worldUp = {0.0f, 1.0f, 0.0f};
    Vec3 right = normalize(cross(forward, worldUp));
    if (length(right) < 1.0e-5f) {
        right = {1.0f, 0.0f, 0.0f};
    }
    Vec3 up = normalize(cross(right, forward));
    Vec3 dir = forward + right * (ndcX * aspect * tanHalf) + up * (ndcY * tanHalf);
    return normalize(dir);
}

bool pickPointOnArenaFloor(
    int fbw,
    int fbh,
    double mx,
    double my,
    const Vec3& eye,
    const Vec3& target,
    float floorY,
    Vec3& out) {
    const Vec3 dir = rayDirFromPixel(fbw, fbh, mx, my, eye, target);
    return rayIntersectPlaneY(eye, dir, floorY, out);
}

bool pointInsideArenaXZ(float x, float z) {
    return std::fabs(x) <= kArenaHalfWidth && std::fabs(z) <= kArenaHalfHeight;
}

void resetLaunchableTopForSlingshot() {
    RigidBody& b = launchableTop();
    b.position = {0.0f, b.radius, 0.0f};
    b.linearVelocity = {0.0f, 0.0f, 0.0f};
    b.angularVelocity = {0.0f, 0.0f, 0.0f};
    b.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
}

void applyBeybladeSphereInertia(RigidBody& body) {
    const float m = body.mass;
    const float r = body.radius;
    const float Ispin = 0.55f * m * r * r;
    const float Itrans = 0.32f * m * r * r;
    body.invInertiaBody = {
        1.0f / std::max(Itrans, 1.0e-6f),
        1.0f / std::max(Ispin, 1.0e-6f),
        1.0f / std::max(Itrans, 1.0e-6f)
    };
}

} // namespace

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

    for (RigidBody& body : g_rigidBodies) {
        applyBeybladeSphereInertia(body);
    }

    std::vector<Vec3> arenaFloorVerts;
    arenaFloorVerts.reserve(5000);
    appendArenaGrid(arenaFloorVerts, kArenaHalfWidth, kArenaHalfHeight, kArenaFloorY, 36);
    appendArenaBorder(arenaFloorVerts, kArenaHalfWidth, kArenaHalfHeight, kArenaFloorY);

    std::vector<Vec3> arenaWallVerts;
    arenaWallVerts.reserve(3000);
    appendArenaWallFrame(arenaWallVerts, kArenaHalfWidth, kArenaHalfHeight, kArenaFloorY, kArenaWallHeight);

    bool s_prevZ = false;
    bool s_zDragActive = false;
    Vec3 s_dragStart{};
    Vec3 s_dragCurrent{};

    while (!glfwWindowShouldClose(window)) {
        const float frameDt = computeDeltaTimeSeconds();
        processInput(window, frameDt);
        updateCameraMouseOrbit(window);
        g_cameraPitch = kIsometricPitchRadians;

        glfwGetFramebufferSize(window, &fbw, &fbh);
        double mx = 0.0;
        double my = 0.0;
        glfwGetCursorPos(window, &mx, &my);

        const Vec3 cameraPos = {
            g_cameraTarget.x + std::cos(g_cameraYaw) * std::cos(g_cameraPitch) * g_cameraDistance,
            g_cameraTarget.y + std::sin(g_cameraPitch) * g_cameraDistance,
            g_cameraTarget.z + std::sin(g_cameraYaw) * std::cos(g_cameraPitch) * g_cameraDistance
        };

        const bool zDown = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;

        Vec3 floorHit{};
        const bool hasFloorHit = fbw > 0 && fbh > 0
            && pickPointOnArenaFloor(fbw, fbh, mx, my, cameraPos, g_cameraTarget, kArenaFloorY, floorHit);

        if (zDown && !s_prevZ) {
            if (hasFloorHit && pointInsideArenaXZ(floorHit.x, floorHit.z)) {
                resetLaunchableTopForSlingshot();
                s_zDragActive = true;
                s_dragStart = floorHit;
                s_dragCurrent = floorHit;
            }
        }

        const bool draggingZLaunch = s_zDragActive && zDown;

        if (draggingZLaunch) {
            if (hasFloorHit) {
                s_dragCurrent = floorHit;
            }
            resetLaunchableTopForSlingshot();
        }

        g_fixedStepAccumulator += frameDt;

        int simulatedSteps = 0;
        while (g_fixedStepAccumulator >= kFixedTimeStepSeconds && simulatedSteps < kMaxFixedStepsPerFrame) {
            physicsFixedSubstep(kFixedTimeStepSeconds, draggingZLaunch);
            g_fixedStepAccumulator -= kFixedTimeStepSeconds;
            simulatedSteps++;
        }
        if (simulatedSteps == kMaxFixedStepsPerFrame && g_fixedStepAccumulator >= kFixedTimeStepSeconds) {
            g_fixedStepAccumulator = 0.0f;
        }

        if (!zDown && s_prevZ && s_zDragActive) {
            RigidBody& b = launchableTop();
            const Vec3 delta = {s_dragCurrent.x - s_dragStart.x, 0.0f, s_dragCurrent.z - s_dragStart.z};
            const float d = length(delta);
            if (d > kLaunchDragMin) {
                const Vec3 dir = delta * (1.0f / d);
                const float speed = std::min(d * kLaunchStrength, kMaxLaunchSpeed);
                // Horizontal impulse only; do not tip the top or add roll/pitch spin.
                b.linearVelocity = {-dir.x * speed, 0.0f, -dir.z * speed};
            }
            b.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
            b.angularVelocity = {0.0f, kTopSpinAboutWorldY, 0.0f};
            s_zDragActive = false;
        }

        s_prevZ = zDown;

        const float aspect = (fbh > 0) ? (static_cast<float>(fbw) / static_cast<float>(fbh)) : 1.0f;
        const Mat4 proj = mat4Perspective(kPerspectiveFovY, aspect, 0.1f, 5000.0f);

        const Mat4 view = mat4LookAt(cameraPos, g_cameraTarget, {0.0f, 1.0f, 0.0f});
        const Mat4 mvp = mat4Multiply(proj, view);

        glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawLineBatch(arenaFloorVerts, GL_LINES, {0.35f, 0.42f, 0.52f}, mvp);
        drawLineBatch(arenaWallVerts, GL_LINES, {0.48f, 0.58f, 0.72f}, mvp);

        for (std::size_t bi = 0; bi < g_rigidBodies.size(); ++bi) {
            const RigidBody& body = g_rigidBodies[bi];
            const bool isLaunchable = (bi == kLaunchableBodyIndex);
            const Vec3 ringRgb = isLaunchable ? Vec3{1.0f, 0.9f, 0.25f} : Vec3{0.45f, 0.92f, 1.0f};

            std::vector<Vec3> marker;
            marker.reserve(kCircleSegments);
            appendCircleXZAtY(
                marker, body.position.x, body.position.y, body.position.z, body.radius, kCircleSegments);
            drawLineBatch(marker, GL_LINE_LOOP, ringRgb, mvp);

            const Vec3 markerCenter = body.position;
            drawVelocityGuides(markerCenter, body.linearVelocity, body.radius, mvp);

            const Vec3 localX = quatRotateVector(body.orientation, {1.0f, 0.0f, 0.0f});
            const Vec3 localY = quatRotateVector(body.orientation, {0.0f, 1.0f, 0.0f});
            const Vec3 localZ = quatRotateVector(body.orientation, {0.0f, 0.0f, 1.0f});
            const float axisLen = body.radius * 1.8f;
            std::vector<Vec3> topAxisX = {markerCenter, markerCenter + localX * axisLen};
            std::vector<Vec3> topAxisY = {markerCenter, markerCenter + localY * axisLen};
            std::vector<Vec3> topAxisZ = {markerCenter, markerCenter + localZ * axisLen};
            drawLineBatch(topAxisX, GL_LINES, {0.95f, 0.35f, 0.35f}, mvp);
            drawLineBatch(topAxisY, GL_LINES, {0.35f, 0.95f, 0.35f}, mvp);
            drawLineBatch(topAxisZ, GL_LINES, {0.35f, 0.65f, 1.0f}, mvp);
        }

        if (draggingZLaunch) {
            std::vector<Vec3> dragPreview;
            dragPreview.reserve(4);
            dragPreview.push_back({s_dragStart.x, kArenaFloorY, s_dragStart.z});
            dragPreview.push_back({s_dragCurrent.x, kArenaFloorY, s_dragCurrent.z});
            const Vec3 delta = {s_dragCurrent.x - s_dragStart.x, 0.0f, s_dragCurrent.z - s_dragStart.z};
            const float d = length(delta);
            if (d > 1.0e-4f) {
                const Vec3 dir = delta * (1.0f / d);
                const Vec3 launchDir = {-dir.x, 0.0f, -dir.z};
                const float arrowLen = std::min(40.0f + 0.15f * d, 180.0f);
                const Vec3 arrowStart = {s_dragCurrent.x, kArenaFloorY, s_dragCurrent.z};
                const Vec3 arrowEnd = {
                    arrowStart.x + launchDir.x * arrowLen,
                    kArenaFloorY,
                    arrowStart.z + launchDir.z * arrowLen};
                dragPreview.push_back(arrowStart);
                dragPreview.push_back(arrowEnd);
            }
            drawLineBatch(dragPreview, GL_LINES, {0.95f, 0.35f, 0.95f}, mvp);
        }

        drawWorldAxesGizmo(view, fbw, fbh);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    shutdownDebugLineRenderer();
    glfwTerminate();
    return 0;
}
