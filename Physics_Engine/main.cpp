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
#include "app/debug_overlay.hpp"

namespace {

constexpr float kPerspectiveFovY = 1.0472f;

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
    Vec3& out) {
    const Vec3 dir = rayDirFromPixel(fbw, fbh, mx, my, eye, target);
    return raycastArenaPlayableFloor(eye, dir, out);
}

void resetLaunchableTopForSlingshot() {
    RigidBody& b = launchableTop();
    const float y = arenaFloorSurfaceYAtXZ(0.0f, 0.0f) + b.colliderBoundingRadius;
    b.position = {0.0f, y, 0.0f};
    b.linearVelocity = {0.0f, 0.0f, 0.0f};
    b.angularVelocity = {0.0f, 0.0f, 0.0f};
    b.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
}

static void rebuildArenaMeshes(std::vector<Vec3>& arenaFloorVerts, std::vector<Vec3>& arenaWallVerts) {
    arenaFloorVerts.clear();
    arenaWallVerts.clear();
    if (g_simTuning.arenaUseParaboloidBowl) {
        arenaFloorVerts.reserve(8000);
        arenaWallVerts.reserve(4000);
        appendBowlParaboloidWireframe(arenaFloorVerts, g_simTuning.arenaFloorY, g_simTuning.bowlCurvatureK,
            g_simTuning.bowlMaxRimRadius, 28, 18);
        appendBowlRimWallFrame(arenaWallVerts, g_simTuning.arenaFloorY, g_simTuning.bowlCurvatureK,
            g_simTuning.bowlMaxRimRadius, g_simTuning.arenaWallHeight, 48);
    } else {
        arenaFloorVerts.reserve(5000);
        appendArenaGrid(arenaFloorVerts, g_simTuning.arenaHalfWidth, g_simTuning.arenaHalfHeight,
            g_simTuning.arenaFloorY, 36);
        appendArenaBorder(arenaFloorVerts, g_simTuning.arenaHalfWidth, g_simTuning.arenaHalfHeight,
            g_simTuning.arenaFloorY);
        arenaWallVerts.reserve(3000);
        appendArenaWallFrame(arenaWallVerts, g_simTuning.arenaHalfWidth, g_simTuning.arenaHalfHeight,
            g_simTuning.arenaFloorY, g_simTuning.arenaWallHeight);
    }
}

static void rebuildArenaIfDirty(std::vector<Vec3>& floor, std::vector<Vec3>& wall) {
    static float w = -1.0f;
    static float h = -1.0f;
    static float fy = 1.0e10f;
    static float wallH = -1.0f;
    static bool bowl = false;
    static float bowlK = -1.0f;
    static float bowlR = -1.0f;
    if (w == g_simTuning.arenaHalfWidth && h == g_simTuning.arenaHalfHeight
        && fy == g_simTuning.arenaFloorY && wallH == g_simTuning.arenaWallHeight
        && bowl == g_simTuning.arenaUseParaboloidBowl && bowlK == g_simTuning.bowlCurvatureK
        && bowlR == g_simTuning.bowlMaxRimRadius) {
        return;
    }
    w = g_simTuning.arenaHalfWidth;
    h = g_simTuning.arenaHalfHeight;
    fy = g_simTuning.arenaFloorY;
    wallH = g_simTuning.arenaWallHeight;
    bowl = g_simTuning.arenaUseParaboloidBowl;
    bowlK = g_simTuning.bowlCurvatureK;
    bowlR = g_simTuning.bowlMaxRimRadius;
    rebuildArenaMeshes(floor, wall);
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
    int fbw = 0;
    int fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);

    if (!initDebugLineRenderer()) {
        glfwTerminate();
        return -1;
    }

    initDebugOverlay(window);

    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.5f);

    initFrameTimer();

    syncRigidBodiesFromTuning();

    std::vector<Vec3> arenaFloorVerts;
    std::vector<Vec3> arenaWallVerts;
    rebuildArenaMeshes(arenaFloorVerts, arenaWallVerts);

    bool s_prevZ = false;
    bool s_prevX = false;
    bool s_zDragActive = false;
    Vec3 s_dragStart{};
    Vec3 s_dragCurrent{};
    float s_launchSpinY = g_simTuning.defaultTopSpinY;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        debugOverlayNewFrame();
        syncRigidBodiesFromTuning();
        rebuildArenaIfDirty(arenaFloorVerts, arenaWallVerts);
        resetStickSlideFrameDebug();
        const float frameDt = computeDeltaTimeSeconds();
        processInput(window, frameDt);
        if (!debugOverlayWantsMouseCapture()) {
            updateCameraMouseOrbit(window);
        }
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
        const bool xDown = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
        if (!debugOverlayWantsKeyboardCapture()) {
            if (xDown && !s_prevX) {
                randomizeTopsForMatch();
                g_fixedStepAccumulator = 0.0f;
                s_zDragActive = false;
                s_launchSpinY = g_simTuning.defaultTopSpinY;
            }
        }
        s_prevX = xDown;

        Vec3 floorHit{};
        const bool hasFloorHit =
            fbw > 0 && fbh > 0 && pickPointOnArenaFloor(fbw, fbh, mx, my, cameraPos, g_cameraTarget, floorHit);

        if (zDown && !s_prevZ) {
            if (hasFloorHit && pointInsideArenaPlayableXZ(floorHit.x, floorHit.z)) {
                resetLaunchableTopForSlingshot();
                s_launchSpinY = g_simTuning.defaultTopSpinY;
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
            if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS) {
                s_launchSpinY -= g_simTuning.launchSpinAdjustRate * frameDt;
            }
            if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) {
                s_launchSpinY += g_simTuning.launchSpinAdjustRate * frameDt;
            }
            s_launchSpinY = std::clamp(s_launchSpinY, -g_simTuning.launchSpinYMax, g_simTuning.launchSpinYMax);
        }

        g_fixedStepAccumulator += frameDt;

        int simulatedSteps = 0;
        while (g_fixedStepAccumulator >= g_simTuning.fixedDt && simulatedSteps < g_simTuning.maxFixedStepsPerFrame) {
            physicsFixedSubstep(g_simTuning.fixedDt, draggingZLaunch);
            g_fixedStepAccumulator -= g_simTuning.fixedDt;
            simulatedSteps++;
        }
        if (simulatedSteps == g_simTuning.maxFixedStepsPerFrame
            && g_fixedStepAccumulator >= g_simTuning.fixedDt) {
            g_fixedStepAccumulator = 0.0f;
        }

        if (!zDown && s_prevZ && s_zDragActive) {
            LaunchIntent intent{};
            intent.spinAboutWorldY = s_launchSpinY;
            const Vec3 delta = {s_dragCurrent.x - s_dragStart.x, 0.0f, s_dragCurrent.z - s_dragStart.z};
            const float d = length(delta);
            if (d > g_simTuning.launchDragMin) {
                const Vec3 dir = delta * (1.0f / d);
                const float speed = std::min(d * g_simTuning.launchStrength, g_simTuning.maxLaunchSpeed);
                intent.horizontalVelocity = {-dir.x * speed, 0.0f, -dir.z * speed};
            }
            applyLaunchIntentFromZSlingshot(launchableTop(), intent);
            s_zDragActive = false;
        }

        s_prevZ = zDown;

        const float aspect = (fbh > 0) ? (static_cast<float>(fbw) / static_cast<float>(fbh)) : 1.0f;
        const Mat4 proj = mat4Perspective(kPerspectiveFovY, aspect, 0.1f, 5000.0f);

        const Mat4 view = mat4LookAt(cameraPos, g_cameraTarget, {0.0f, 1.0f, 0.0f});
        const Mat4 mvp = mat4Multiply(proj, view);

        glViewport(0, 0, fbw, fbh);
        glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawLineBatch(arenaFloorVerts, GL_LINES, {0.35f, 0.42f, 0.52f}, mvp);
        drawLineBatch(arenaWallVerts, GL_LINES, {0.48f, 0.58f, 0.72f}, mvp);

        for (std::size_t bi = 0; bi < g_rigidBodies.size(); ++bi) {
            const RigidBody& body = g_rigidBodies[bi];
            const bool isLaunchable = (bi == kLaunchableBodyIndex);
            const Vec3 ringRgb = isLaunchable ? Vec3{1.0f, 0.9f, 0.25f} : Vec3{0.45f, 0.92f, 1.0f};
            const Vec3 tipRgb = isLaunchable ? Vec3{1.0f, 0.62f, 0.12f} : Vec3{0.55f, 0.88f, 1.0f};
            const Vec3 diskRgb = isLaunchable ? Vec3{0.98f, 0.95f, 0.4f} : Vec3{0.5f, 0.82f, 1.0f};
            const Vec3 attackRgb = isLaunchable ? Vec3{1.0f, 0.45f, 0.15f} : Vec3{0.35f, 0.72f, 1.0f};
            const Vec3 diskOutlineRgb = isLaunchable ? Vec3{0.72f, 0.68f, 0.28f} : Vec3{0.32f, 0.58f, 0.82f};
            constexpr int kWireSphereSegments = 48;

            const float diskY =
                (bi == 0) ? g_simTuning.body0DiskRingY : g_simTuning.body1DiskRingY;
            const float diskRadial =
                (bi == 0) ? g_simTuning.body0DiskRadial : g_simTuning.body1DiskRadial;
            if (diskRadial > 0.5f) {
                drawCircleBodyXZPlane(
                    body.position, body.orientation, diskY, diskRadial, kCircleSegments, diskOutlineRgb, mvp);
            }

            for (int si = 0; si < static_cast<int>(body.collisionSphereCount); ++si) {
                const CollisionSphere& sp = body.collisionSpheres[si];
                const Vec3 wc =
                    body.position + quatRotateVector(body.orientation, sp.offsetBody);
                Vec3 rgb = ringRgb;
                switch (sp.role) {
                case CollisionSphereRole::Tip:
                    rgb = tipRgb;
                    break;
                case CollisionSphereRole::Hub:
                    rgb = ringRgb;
                    break;
                case CollisionSphereRole::DiskRing:
                    rgb = diskRgb;
                    break;
                case CollisionSphereRole::MidAttack:
                    rgb = attackRgb;
                    break;
                default:
                    rgb = ringRgb;
                    break;
                }
                drawWireframeSphereWorldAxes(wc, sp.radius, kWireSphereSegments, rgb, mvp);

                const bool strutToCom = (sp.role != CollisionSphereRole::Hub)
                    && (length(sp.offsetBody) > 1.0e-4f);
                if (strutToCom) {
                    const Vec3 strutRgb = {
                        rgb.x * 0.38f,
                        rgb.y * 0.38f,
                        rgb.z * 0.38f,
                    };
                    const std::vector<Vec3> strut = {body.position, wc};
                    drawLineBatch(strut, GL_LINES, strutRgb, mvp);
                }
            }

            const Vec3 markerCenter = body.position;
            drawVelocityGuides(markerCenter, body.linearVelocity, body.colliderBoundingRadius, mvp);

            const Vec3 localX = quatRotateVector(body.orientation, {1.0f, 0.0f, 0.0f});
            const Vec3 localY = quatRotateVector(body.orientation, {0.0f, 1.0f, 0.0f});
            const Vec3 localZ = quatRotateVector(body.orientation, {0.0f, 0.0f, 1.0f});
            const float axisLen = body.colliderBoundingRadius * 1.8f;
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
            const float yStart = arenaFloorSurfaceYAtXZ(s_dragStart.x, s_dragStart.z);
            const float yCur = arenaFloorSurfaceYAtXZ(s_dragCurrent.x, s_dragCurrent.z);
            dragPreview.push_back({s_dragStart.x, yStart, s_dragStart.z});
            dragPreview.push_back({s_dragCurrent.x, yCur, s_dragCurrent.z});
            const Vec3 delta = {s_dragCurrent.x - s_dragStart.x, 0.0f, s_dragCurrent.z - s_dragStart.z};
            const float d = length(delta);
            if (d > 1.0e-4f) {
                const Vec3 dir = delta * (1.0f / d);
                const Vec3 launchDir = {-dir.x, 0.0f, -dir.z};
                const float arrowLen = std::min(40.0f + 0.15f * d, 180.0f);
                const Vec3 arrowStart = {s_dragCurrent.x, yCur, s_dragCurrent.z};
                const Vec3 arrowEnd = {
                    arrowStart.x + launchDir.x * arrowLen,
                    yCur,
                    arrowStart.z + launchDir.z * arrowLen};
                dragPreview.push_back(arrowStart);
                dragPreview.push_back(arrowEnd);
            }
            drawLineBatch(dragPreview, GL_LINES, {0.95f, 0.35f, 0.95f}, mvp);
        }

        debugOverlayDrawHud(
            frameDt,
            simulatedSteps,
            g_fixedStepAccumulator,
            draggingZLaunch,
            s_launchSpinY);
        debugOverlayDrawTuningPanel();
        debugOverlayDrawOrientationGizmo(view, fbw, fbh);
        debugOverlayRenderDrawData();

        glfwSwapBuffers(window);
    }

    shutdownDebugOverlay();
    shutdownDebugLineRenderer();
    glfwTerminate();
    return 0;
}
