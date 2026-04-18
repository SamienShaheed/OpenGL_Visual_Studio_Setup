#include "app/debug_overlay.hpp"
#include "app/camera.hpp"
#include "math/math.hpp"
#include "physics/physics.hpp"

#include <cmath>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define IMGUI_IMPL_OPENGL_LOADER_GLAD
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

void initDebugOverlay(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 400 core");

    glfwSetScrollCallback(window, [](GLFWwindow* w, double x, double y) {
        ImGui_ImplGlfw_ScrollCallback(w, x, y);
        if (!ImGui::GetIO().WantCaptureMouse) {
            mouseScrollCallback(w, x, y);
        }
    });
}

void shutdownDebugOverlay() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void debugOverlayNewFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void debugOverlayDrawHud(
    float frameDt,
    int physicsSubstepsThisFrame,
    float fixedStepAccumulator,
    bool draggingZLaunch,
    float launchSpinY) {
    ImGuiIO& io = ImGui::GetIO();

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 14.0f, 14.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.88f);

    constexpr ImGuiWindowFlags kFlags =
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::Begin("Debug##physics", nullptr, kFlags);

    const float fps = (frameDt > 1.0e-6f) ? (1.0f / frameDt) : 0.0f;
    ImGui::Text("FPS (frame):     %.0f", static_cast<double>(fps));
    ImGui::Text("Frame dt:        %.3f ms", static_cast<double>(frameDt * 1000.0f));
    ImGui::Text("ImGui FPS:       %.0f", static_cast<double>(io.Framerate));
    ImGui::Separator();
    ImGui::Text("Fixed dt:        %.4f s", static_cast<double>(g_simTuning.fixedDt));
    ImGui::Text(
        "Physics substeps:%d (max %d)", physicsSubstepsThisFrame, g_simTuning.maxFixedStepsPerFrame);
    ImGui::Text("Accumulator:     %.4f s", static_cast<double>(fixedStepAccumulator));
    ImGui::Separator();

    for (std::size_t i = 0; i < g_rigidBodies.size(); ++i) {
        const RigidBody& b = g_rigidBodies[i];
        const float speed = length(b.linearVelocity);
        const float wx = b.angularVelocity.x;
        const float wy = b.angularVelocity.y;
        const float wz = b.angularVelocity.z;
        const float omegaLen = length(b.angularVelocity);

        if (i > 0) {
            ImGui::Separator();
        }
        ImGui::Text("Body %u%s", static_cast<unsigned>(i), (i == kLaunchableBodyIndex) ? " (launch)" : "");
        ImGui::Text("  pos   (%.1f, %.1f, %.1f)", static_cast<double>(b.position.x),
            static_cast<double>(b.position.y), static_cast<double>(b.position.z));
        ImGui::Text("  |v|   %.2f", static_cast<double>(speed));
        ImGui::Text("  v     (%.2f, %.2f, %.2f)", static_cast<double>(b.linearVelocity.x),
            static_cast<double>(b.linearVelocity.y), static_cast<double>(b.linearVelocity.z));
        ImGui::Text("  |omega| %.2f rad/s", static_cast<double>(omegaLen));
        ImGui::Text("  omega (%.2f, %.2f, %.2f)", static_cast<double>(wx), static_cast<double>(wy),
            static_cast<double>(wz));
        {
            // While Z-dragging, the launch body is reset every substep so ω is zero; use pending launch spin.
            const float wyForDir =
                (i == kLaunchableBodyIndex && draggingZLaunch) ? launchSpinY : wy;
            const char* spinDirWy = "none";
            if (wyForDir > 1.0e-4f) {
                spinDirWy = "CCW (view from +Y)";
            } else if (wyForDir < -1.0e-4f) {
                spinDirWy = "CW (view from +Y)";
            }
            if (i == kLaunchableBodyIndex && draggingZLaunch) {
                ImGui::Text("  spin dir (+Y): %s (pending launch)", spinDirWy);
            } else {
                ImGui::Text("  spin dir (+Y): %s", spinDirWy);
            }
        }
    }

    ImGui::Separator();
    ImGui::Text("Z-launch drag:   %s", draggingZLaunch ? "yes" : "no");
    ImGui::Text("Launch spin wy:  %.2f rad/s ( [ ] )", static_cast<double>(launchSpinY));
    ImGui::TextDisabled("X: random match (positions, spins, head-on)");

    ImGui::End();
}

void debugOverlayDrawTuningPanel() {
    ImGui::SetNextWindowPos(ImVec2(14.0f, 14.0f), ImGuiCond_FirstUseEver, ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.88f);
    constexpr ImGuiWindowFlags kFlags =
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::Begin("Tuning##physics", nullptr, kFlags);

    ImGui::SeparatorText("Simulation");
    ImGui::DragFloat3("Gravity", &g_simTuning.gravity.x, 2.0f, 0.0f, 0.0f, "%.1f");
    ImGui::SliderFloat("Floor/wall restitution", &g_simTuning.contactRestitution, 0.0f, 1.0f);
    ImGui::SliderFloat("Floor friction mu", &g_simTuning.frictionMuFloor, 0.0f, 2.0f);
    ImGui::SliderFloat("Sphere restitution", &g_simTuning.sphereSphereRestitution, 0.0f, 1.0f);
    ImGui::SliderFloat("Sphere friction mu", &g_simTuning.sphereSphereFrictionMu, 0.0f, 2.0f);
    ImGui::SliderInt("Sphere-sphere iterations", &g_simTuning.sphereSphereIterations, 1, 16);
    ImGui::DragFloat("SS slip ref (m/s)", &g_simTuning.sphereSphereSlipRef, 1.0f, 5.0f, 200.0f);
    ImGui::SliderFloat("SS slip boost", &g_simTuning.sphereSphereSlipBoost, 0.0f, 2.0f);
    ImGui::DragFloat("SS slip norm cap", &g_simTuning.sphereSphereSlipBoostMax, 0.1f, 0.5f, 8.0f);
    ImGui::DragFloat("SS seek accel", &g_simTuning.sphereSphereSeekAccel, 0.5f, 0.0f, 80.0f);
    ImGui::DragFloat("SS impulse scale", &g_simTuning.sphereSphereImpulseResponseScale, 0.02f, 0.2f, 1.5f);
    ImGui::TextUnformatted("Caps (0 = off):");
    ImGui::DragFloat("Max |v|", &g_simTuning.maxLinearSpeed, 10.0f, 0.0f, 4000.0f);
    ImGui::DragFloat("Max |omega|", &g_simTuning.maxAngularSpeed, 2.0f, 0.0f, 800.0f);
    ImGui::DragFloat("Max upward vy", &g_simTuning.maxUpwardLinearSpeed, 5.0f, 0.0f, 2000.0f);

    ImGui::SeparatorText("Arena");
    ImGui::DragFloat("Half width (X)", &g_simTuning.arenaHalfWidth, 1.0f, 50.0f, 2000.0f);
    ImGui::DragFloat("Half depth (Z)", &g_simTuning.arenaHalfHeight, 1.0f, 50.0f, 2000.0f);
    ImGui::DragFloat("Floor Y", &g_simTuning.arenaFloorY, 0.5f, -500.0f, 500.0f);
    ImGui::DragFloat("Wall height (visual)", &g_simTuning.arenaWallHeight, 1.0f, 0.0f, 2000.0f);

    ImGui::SeparatorText("Time step");
    float fixedDt = g_simTuning.fixedDt;
    if (ImGui::SliderFloat("Fixed dt (s)", &fixedDt, 1.0f / 240.0f, 1.0f / 30.0f, "%.5f")) {
        g_simTuning.fixedDt = fixedDt;
    }
    ImGui::SliderInt("Max substeps / frame", &g_simTuning.maxFixedStepsPerFrame, 1, 32);

    ImGui::SeparatorText("Z-launch");
    ImGui::DragFloat("Default spin wy", &g_simTuning.defaultTopSpinY, 0.25f, -80.0f, 80.0f);
    ImGui::DragFloat("Launch spin max abs", &g_simTuning.launchSpinYMax, 0.5f, 1.0f, 120.0f);
    ImGui::DragFloat("Launch drag min", &g_simTuning.launchDragMin, 0.05f, 0.05f, 20.0f);
    ImGui::DragFloat("Launch strength", &g_simTuning.launchStrength, 0.05f, 0.1f, 20.0f);
    ImGui::DragFloat("Max launch speed", &g_simTuning.maxLaunchSpeed, 5.0f, 10.0f, 5000.0f);
    ImGui::DragFloat("Spin adjust (rad/s/s)", &g_simTuning.launchSpinAdjustRate, 0.5f, 0.5f, 200.0f);

    ImGui::SeparatorText("Bodies");
    ImGui::DragFloat("Body 0 mass", &g_simTuning.body0Mass, 0.05f, 0.05f, 100.0f);
    ImGui::DragFloat("Body 1 mass", &g_simTuning.body1Mass, 0.05f, 0.05f, 100.0f);
    ImGui::DragFloat("Body 0 radius", &g_simTuning.body0Radius, 0.5f, 1.0f, 300.0f);
    ImGui::DragFloat("Body 1 radius", &g_simTuning.body1Radius, 0.5f, 1.0f, 300.0f);

    ImGui::SeparatorText("Beyblade compound (tip + hub + disk ring)");
    ImGui::TextUnformatted("+Y = spin axis. Tip below COM; disk ring above COM (sphere approximation).");
    ImGui::SeparatorText("Body 0 shape");
    ImGui::DragFloat("Tip radius", &g_simTuning.body0TipRadius, 0.25f, 0.0f, 80.0f);
    ImGui::DragFloat("Tip below COM", &g_simTuning.body0TipOffsetBelowCom, 0.5f, 0.0f, 120.0f);
    ImGui::Checkbox("Use hub sphere", &g_simTuning.body0UseHubCollider);
    ImGui::DragFloat("Hub radius", &g_simTuning.body0HubColliderRadius, 0.5f, 1.0f, 120.0f);
    ImGui::DragFloat("Disk ring Y", &g_simTuning.body0DiskRingY, 0.5f, -40.0f, 120.0f);
    ImGui::DragFloat("Disk radial", &g_simTuning.body0DiskRadial, 0.5f, 0.0f, 200.0f);
    ImGui::DragFloat("Disk sphere R", &g_simTuning.body0DiskSphereRadius, 0.25f, 2.0f, 80.0f);
    ImGui::DragInt("Disk sphere count", &g_simTuning.body0DiskSphereCount, 1, 0, 12);
    ImGui::DragInt("Mid attack count", &g_simTuning.body0MidAttackCount, 1, 0, 8);
    ImGui::DragFloat("Mid attack R", &g_simTuning.body0MidAttackRadius, 0.25f, 2.0f, 80.0f);
    ImGui::DragFloat("Mid attack radial", &g_simTuning.body0MidAttackRadial, 0.5f, 0.0f, 200.0f);
    ImGui::DragFloat("Mid attack Y", &g_simTuning.body0MidAttackY, 0.5f, -60.0f, 80.0f);
    ImGui::SeparatorText("Body 1 shape");
    ImGui::DragFloat("B1 tip radius", &g_simTuning.body1TipRadius, 0.25f, 0.0f, 80.0f);
    ImGui::DragFloat("B1 tip below COM", &g_simTuning.body1TipOffsetBelowCom, 0.5f, 0.0f, 120.0f);
    ImGui::Checkbox("B1 use hub sphere", &g_simTuning.body1UseHubCollider);
    ImGui::DragFloat("B1 hub radius", &g_simTuning.body1HubColliderRadius, 0.5f, 1.0f, 120.0f);
    ImGui::DragFloat("B1 disk ring Y", &g_simTuning.body1DiskRingY, 0.5f, -40.0f, 120.0f);
    ImGui::DragFloat("B1 disk radial", &g_simTuning.body1DiskRadial, 0.5f, 0.0f, 200.0f);
    ImGui::DragFloat("B1 disk sphere R", &g_simTuning.body1DiskSphereRadius, 0.25f, 2.0f, 80.0f);
    ImGui::DragInt("B1 disk sphere count", &g_simTuning.body1DiskSphereCount, 1, 0, 12);
    ImGui::DragInt("B1 mid attack count", &g_simTuning.body1MidAttackCount, 1, 0, 8);
    ImGui::DragFloat("B1 mid attack R", &g_simTuning.body1MidAttackRadius, 0.25f, 2.0f, 80.0f);
    ImGui::DragFloat("B1 mid attack radial", &g_simTuning.body1MidAttackRadial, 0.5f, 0.0f, 200.0f);
    ImGui::DragFloat("B1 mid attack Y", &g_simTuning.body1MidAttackY, 0.5f, -60.0f, 80.0f);

    ImGui::SeparatorText("Inertia (sphere model)");
    ImGui::DragFloat("Spin factor", &g_simTuning.inertiaSpinFactor, 0.01f, 0.05f, 1.0f);
    ImGui::DragFloat("Trans factor", &g_simTuning.inertiaTransFactor, 0.01f, 0.05f, 1.0f);

    ImGui::SeparatorText("Damping / drag");
    ImGui::DragFloat("Angular damping (/s)", &g_simTuning.angularDampingPerSecond, 0.02f, 0.0f, 5.0f);
    ImGui::DragFloat("Spin-axis damp scale", &g_simTuning.angularDampingSpinAxisScale, 0.02f, 0.0f, 2.0f);
    ImGui::TextUnformatted("(Lower = spin about body axis decays slower than wobble.)");
    ImGui::DragFloat("Linear drag XZ (/s)", &g_simTuning.linearAirDragPerSecond, 0.02f, 0.0f, 3.0f);
    ImGui::DragFloat("Linear drag Y (/s)", &g_simTuning.linearVerticalAirDragPerSecond, 0.02f, 0.0f, 3.0f);

    ImGui::SeparatorText("Gyro upright + battle band");
    ImGui::DragFloat("Gyro strength", &g_simTuning.gyroUprightStrength, 5.0f, 0.0f, 20000.0f);
    ImGui::DragFloat("Gyro spin dead", &g_simTuning.gyroUprightSpinDead, 0.1f, 0.0f, 40.0f);
    ImGui::DragFloat("Gyro spin full gate", &g_simTuning.gyroUprightSpinFull, 0.25f, 0.5f, 80.0f);
    ImGui::DragFloat("Gyro max torque", &g_simTuning.gyroUprightMaxTorque, 20.0f, 0.0f, 50000.0f);
    ImGui::DragFloat("Band max impulse / step", &g_simTuning.battleBandMaxImpulsePerStep, 1.0f, 0.0f, 500.0f);
    ImGui::DragFloat("Band rest XZ dist", &g_simTuning.battleBandRestDistance, 1.0f, 0.0f, 400.0f);
    ImGui::DragFloat("Band stiffness", &g_simTuning.battleBandStiffness, 0.5f, 0.0f, 400.0f);
    ImGui::DragFloat("Band min spin (axis)", &g_simTuning.battleBandMinSpinAboutAxis, 0.1f, 0.0f, 40.0f);

    ImGui::SeparatorText("X random match");
    ImGui::DragFloat("Closing speed", &g_simTuning.matchInitialClosingSpeed, 5.0f, 20.0f, 800.0f);
    ImGui::DragInt("Approach grace (substeps)", &g_simTuning.matchApproachGraceSubsteps, 1, 0, 600);
    ImGui::SliderFloat("Approach friction scale", &g_simTuning.matchApproachFrictionScale, 0.05f, 1.0f);
    ImGui::DragFloat("Spin abs max (rad/s)", &g_simTuning.matchSpinAbsMax, 1.0f, 0.0f, 80.0f);
    ImGui::DragFloat("Spin tilt max (rad/s)", &g_simTuning.matchSpinTiltMax, 0.25f, 0.0f, 30.0f);
    ImGui::DragFloat("Arena inset", &g_simTuning.matchArenaInset, 1.0f, 0.0f, 80.0f);
    ImGui::DragFloat("Min sep extra", &g_simTuning.matchMinSeparationExtra, 1.0f, 0.0f, 200.0f);

    if (ImGui::Button("Reset all to defaults")) {
        resetSimulationTuningToDefaults();
        syncRigidBodiesFromTuning();
    }

    ImGui::End();
}

void debugOverlayRenderDrawData() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool debugOverlayWantsMouseCapture() {
    return ImGui::GetIO().WantCaptureMouse;
}

bool debugOverlayWantsKeyboardCapture() {
    return ImGui::GetIO().WantCaptureKeyboard;
}
