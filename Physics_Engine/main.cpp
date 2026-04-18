#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstring>
#include <iostream>

// --- Window -----------------------------------------------------------------
int WINDOW_WIDTH = 800;
int WINDOW_HEIGHT = 600;

// --- Bouncing ball
struct Vec2 {
    float x, y;
};

const int RADIUS = 50;    // Ball radius in world units (orthographic space).
const int SEGMENTS = 100; // Number of line segments used to draw the circle.

Vec2 VELOCITY = { -400.0f, -350.0f };  // Ball velocity in world units per second.
Vec2 ballPosition = { 0.0f, 0.0f };    // Ball center position in world coordinates.

double g_lastFrameTime = 0.0;              // Last sampled GLFW time, used for delta-time.
bool g_frameTimerInitialized = false;      // Ensures first frame starts with a safe dt.
constexpr double kMaxDeltaTimeSeconds = 0.1; // Clamp to avoid huge simulation jumps.

// --- OpenGL (minimal shader + dynamic line loop for circle outline) ---------
GLuint g_shaderProgram = 0; // Linked program used for drawing the circle.
GLint g_uProjectionLoc = -1; // Cached uniform location for projection matrix.
GLuint g_vao = 0; // Vertex array object describing circle vertex layout.
GLuint g_vbo = 0; // Dynamic vertex buffer updated each frame with circle points.

static const char* kVertSrc = R"(
#version 400 core
layout (location = 0) in vec2 aPos;
uniform mat4 uProjection;
void main() {
    gl_Position = uProjection * vec4(aPos, 0.0, 1.0);
}
)";

static const char* kFragSrc = R"(
#version 400 core
out vec4 FragColor;
void main() {
    FragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
)";

static void orthoRh(float left, float right, float bottom, float top, float nearVal, float farVal, float* out16) {
    const float rl = right - left;
    const float tb = top - bottom;
    const float fn = farVal - nearVal;
    std::memset(out16, 0, 16 * sizeof(float));
    out16[0] = 2.0f / rl;
    out16[5] = 2.0f / tb;
    out16[10] = -2.0f / fn;
    out16[12] = -(right + left) / rl;
    out16[13] = -(top + bottom) / tb;
    out16[14] = -(farVal + nearVal) / fn;
    out16[15] = 1.0f;
}

static GLuint compileShader(GLenum type, const char* src) {
    GLuint id = glCreateShader(type);
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);
    GLint ok = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(id, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error: " << log << std::endl;
        glDeleteShader(id);
        return 0;
    }
    return id;
}

static GLuint createProgram(const char* vs, const char* fs) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) {
        return 0;
    }
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::cerr << "Program link error: " << log << std::endl;
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

static void fillCircleVertices(float centerX, float centerY, float radius, int numSegments, float* outXY, int maxFloats) {
    const int needed = numSegments * 2;
    if (needed > maxFloats) {
        return;
    }
    const float angleStep = 3.1415926f * 2.0f / static_cast<float>(numSegments);
    const float tangentLength = tanf(angleStep);
    const float radiusCorrection = cosf(angleStep);

    float currentX = radius;
    float currentY = 0.0f;

    for (int i = 0; i < numSegments; ++i) {
        outXY[i * 2 + 0] = currentX + centerX;
        outXY[i * 2 + 1] = currentY + centerY;

        const float tangentX = -currentY;
        const float tangentY = currentX;
        currentX += tangentX * tangentLength;
        currentY += tangentY * tangentLength;
        currentX *= radiusCorrection;
        currentY *= radiusCorrection;
    }
}

void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

float halfWidthFromFramebuffer(int fbw) {
    return static_cast<float>(fbw) * 0.5f;
}

float halfHeightFromFramebuffer(int fbh) {
    return static_cast<float>(fbh) * 0.5f;
}

int checkCollisionForX(float x, float halfW) {
    const float leftWall = -halfW;
    const float rightWall = halfW;
    const float leftEdge = x - static_cast<float>(RADIUS);
    const float rightEdge = x + static_cast<float>(RADIUS);

    if (rightEdge >= rightWall || leftEdge <= leftWall) {
        return -1;
    }
    return 1;
}

int checkCollisionForY(float y, float halfH) {
    const float topWall = halfH;
    const float bottomWall = -halfH;
    const float topEdge = y + static_cast<float>(RADIUS);
    const float bottomEdge = y - static_cast<float>(RADIUS);

    if (topEdge >= topWall || bottomEdge <= bottomWall) {
        return -1;
    }
    return 1;
}

float correctXPosition(float x, float halfW) {
    const float leftWall = -halfW;
    const float rightWall = halfW;
    const float leftEdge = x - static_cast<float>(RADIUS);
    const float rightEdge = x + static_cast<float>(RADIUS);

    if (leftEdge < leftWall) {
        const float overlap = leftWall - leftEdge;
        return x + overlap;
    }
    if (rightEdge > rightWall) {
        const float overlap = rightEdge - rightWall;
        return x - overlap;
    }
    return x;
}

float correctYPosition(float y, float halfH) {
    const float topWall = halfH;
    const float bottomWall = -halfH;
    const float topEdge = y + static_cast<float>(RADIUS);
    const float bottomEdge = y - static_cast<float>(RADIUS);

    if (topEdge > topWall) {
        const float overlap = topEdge - topWall;
        return y - overlap;
    }
    if (bottomEdge < bottomWall) {
        const float overlap = bottomWall - bottomEdge;
        return y + overlap;
    }
    return y;
}

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

    // Guard against clock anomalies and large hitch spikes.
    if (!std::isfinite(deltaTime) || deltaTime < 0.0) {
        deltaTime = 0.0;
    } else if (deltaTime > kMaxDeltaTimeSeconds) {
        deltaTime = kMaxDeltaTimeSeconds;
    }

    return static_cast<float>(deltaTime);
}

void updateBall(float halfW, float halfH, float dt) {
    ballPosition.x += VELOCITY.x * dt;
    ballPosition.y += VELOCITY.y * dt;

    if (checkCollisionForX(ballPosition.x, halfW) == -1) {
        ballPosition.x = correctXPosition(ballPosition.x, halfW);
        VELOCITY.x *= -1.0f;
    }
    if (checkCollisionForY(ballPosition.y, halfH) == -1) {
        ballPosition.y = correctYPosition(ballPosition.y, halfH);
        VELOCITY.y *= -1.0f;
    }
}

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Physics Engine", nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    int fbw = 0;
    int fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);

    g_shaderProgram = createProgram(kVertSrc, kFragSrc);
    if (g_shaderProgram == 0) {
        return -1;
    }
    g_uProjectionLoc = glGetUniformLocation(g_shaderProgram, "uProjection");

    glGenVertexArrays(1, &g_vao);
    glGenBuffers(1, &g_vbo);
    glBindVertexArray(g_vao);
    glBindBuffer(GL_ARRAY_BUFFER, g_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(SEGMENTS * 2 * sizeof(float)), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    initFrameTimer();

    float circleVerts[SEGMENTS * 2];

    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        glfwGetFramebufferSize(window, &fbw, &fbh);
        const float halfW = halfWidthFromFramebuffer(fbw);
        const float halfH = halfHeightFromFramebuffer(fbh);

        const float dtSeconds = computeDeltaTimeSeconds();
        updateBall(halfW, halfH, dtSeconds);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        float proj[16];
        orthoRh(-halfW, halfW, -halfH, halfH, -1.0f, 1.0f, proj);

        fillCircleVertices(ballPosition.x, ballPosition.y, static_cast<float>(RADIUS), SEGMENTS, circleVerts,
                           SEGMENTS * 2);

        glUseProgram(g_shaderProgram);
        glUniformMatrix4fv(g_uProjectionLoc, 1, GL_FALSE, proj);

        glBindVertexArray(g_vao);
        glBindBuffer(GL_ARRAY_BUFFER, g_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(SEGMENTS * 2 * sizeof(float)), circleVerts);
        glDrawArrays(GL_LINE_LOOP, 0, SEGMENTS);

        glBindVertexArray(0);
        glUseProgram(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteBuffers(1, &g_vbo);
    glDeleteVertexArrays(1, &g_vao);
    glDeleteProgram(g_shaderProgram);

    glfwTerminate();
    return 0;
}
