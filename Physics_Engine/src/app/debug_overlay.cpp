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
    ImGui::Text("Fixed dt:        %.4f s", static_cast<double>(kFixedTimeStepSeconds));
    ImGui::Text("Physics substeps:%d (max %d)", physicsSubstepsThisFrame, kMaxFixedStepsPerFrame);
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

    ImGui::End();
}

void debugOverlayRenderDrawData() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool debugOverlayWantsMouseCapture() {
    return ImGui::GetIO().WantCaptureMouse;
}
