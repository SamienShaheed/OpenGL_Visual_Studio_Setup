#include "app/app.hpp"

#include <GLFW/glfw3.h>

#include <cmath>

int WINDOW_WIDTH = 1000;
int WINDOW_HEIGHT = 700;

namespace {
constexpr double kMaxDeltaTimeSeconds = 0.1;
double g_lastFrameTime = 0.0;
bool g_frameTimerInitialized = false;
} // namespace

void initFrameTimer() {
    g_lastFrameTime = glfwGetTime();
    g_frameTimerInitialized = true;
}

float computeDeltaTimeSeconds() {
    const double currentTime = glfwGetTime();
    if (!g_frameTimerInitialized) {
        g_lastFrameTime = currentTime;
        g_frameTimerInitialized = true;
        return 0.0f;
    }

    double deltaTime = currentTime - g_lastFrameTime;
    g_lastFrameTime = currentTime;

    if (!std::isfinite(deltaTime) || deltaTime < 0.0) {
        deltaTime = 0.0;
    } else if (deltaTime > kMaxDeltaTimeSeconds) {
        deltaTime = kMaxDeltaTimeSeconds;
    }
    return static_cast<float>(deltaTime);
}
