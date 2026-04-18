#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

// --- Window -----------------------------------------------------------------
int WINDOW_WIDTH = 1000;      // Initial window width in screen pixels.
int WINDOW_HEIGHT = 700;      // Initial window height in screen pixels.

// --- Basic math --------------------------------------------------------------
struct Vec2 {
    float x, y;
};

struct Vec3 {
    float x, y, z;
};

struct Mat4 {
    float m[16];
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }

float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
Vec3 normalize(const Vec3& v) {
    const float len = length(v);
    if (len <= 0.00001f) {
        return {0.0f, 0.0f, 0.0f};
    }
    return v * (1.0f / len);
}

Mat4 mat4Identity() {
    return {{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    }};
}

Mat4 mat4Multiply(const Mat4& a, const Mat4& b) {
    Mat4 out = {};
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            out.m[col * 4 + row] =
                a.m[0 * 4 + row] * b.m[col * 4 + 0] +
                a.m[1 * 4 + row] * b.m[col * 4 + 1] +
                a.m[2 * 4 + row] * b.m[col * 4 + 2] +
                a.m[3 * 4 + row] * b.m[col * 4 + 3];
        }
    }
    return out;
}

Mat4 mat4Perspective(float fovYRadians, float aspect, float zNear, float zFar) {
    Mat4 out = {};
    const float f = 1.0f / std::tan(fovYRadians * 0.5f);
    out.m[0] = f / aspect;
    out.m[5] = f;
    out.m[10] = (zFar + zNear) / (zNear - zFar);
    out.m[11] = -1.0f;
    out.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
    return out;
}

Mat4 mat4LookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalize(center - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);

    Mat4 out = mat4Identity();
    out.m[0] = s.x;
    out.m[1] = u.x;
    out.m[2] = -f.x;
    out.m[4] = s.y;
    out.m[5] = u.y;
    out.m[6] = -f.y;
    out.m[8] = s.z;
    out.m[9] = u.z;
    out.m[10] = -f.z;
    out.m[12] = -dot(s, eye);
    out.m[13] = -dot(u, eye);
    out.m[14] = dot(f, eye);
    return out;
}

// --- Bouncing marker simulation (2D logic, visualized in 3D) ----------------
constexpr float kArenaHalfWidth = 450.0f;   // Half arena width in world units.
constexpr float kArenaHalfHeight = 300.0f;  // Half arena depth in world units.
constexpr float kMarkerRadius = 45.0f;      // Marker radius in world units.
constexpr int kCircleSegments = 64;         // Circle smoothness for marker draw.

Vec2 g_velocity = {-400.0f, -350.0f};       // Marker velocity in world units per second.
Vec2 g_position = {0.0f, 0.0f};             // Marker center in arena plane coordinates.

double g_lastFrameTime = 0.0;               // Last sampled GLFW time.
bool g_frameTimerInitialized = false;       // First-frame timer guard.
constexpr double kMaxDeltaTimeSeconds = 0.1;// Clamp to avoid huge simulation jumps.

// --- Camera controls ---------------------------------------------------------
constexpr float kIsometricPitchRadians = 0.6154797f; // Fixed isometric pitch (~35.264 deg).
float g_cameraYaw = 0.7853982f;             // Horizontal orbit angle in radians (45 deg for isometric feel).
float g_cameraPitch = kIsometricPitchRadians; // Vertical orbit is locked to isometric pitch.
float g_cameraDistance = 900.0f;            // Orbit radius from target.
Vec3 g_cameraTarget = {0.0f, 0.0f, 0.0f};  // Point camera looks at.
bool g_isRotatingCamera = false;            // True while left mouse is dragging orbit.
double g_lastMouseX = 0.0;                  // Last cursor X used for mouse orbit.
double g_lastMouseY = 0.0;                  // Last cursor Y used for mouse orbit.

// --- OpenGL debug line pipeline ---------------------------------------------
GLuint g_shaderProgram = 0;                 // Linked shader program.
GLint g_uMvpLoc = -1;                       // MVP uniform location.
GLint g_uColorLoc = -1;                     // Color uniform location.
GLuint g_lineVao = 0;                       // VAO for line vertices.
GLuint g_lineVbo = 0;                       // Dynamic VBO for debug lines.
constexpr int kMaxDebugVertices = 12000;    // Dynamic line buffer capacity.

static const char* kVertSrc = R"(
#version 400 core
layout (location = 0) in vec3 aPos;
uniform mat4 uMVP;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

static const char* kFragSrc = R"(
#version 400 core
uniform vec3 uColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
    glViewport(0, 0, width, height);
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

    if (!std::isfinite(deltaTime) || deltaTime < 0.0) {
        deltaTime = 0.0;
    } else if (deltaTime > kMaxDeltaTimeSeconds) {
        deltaTime = kMaxDeltaTimeSeconds;
    }
    return static_cast<float>(deltaTime);
}

GLuint compileShader(GLenum type, const char* src) {
    const GLuint id = glCreateShader(type);
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);
    GLint ok = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(id, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error: " << log << '\n';
        glDeleteShader(id);
        return 0;
    }
    return id;
}

GLuint createProgram(const char* vs, const char* fs) {
    const GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    const GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) {
        return 0;
    }

    const GLuint p = glCreateProgram();
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
        std::cerr << "Program link error: " << log << '\n';
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

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
    // Scroll up to zoom in, down to zoom out.
    g_cameraDistance -= static_cast<float>(yOffset) * 40.0f;
    g_cameraDistance = std::clamp(g_cameraDistance, 300.0f, 1800.0f);
}

int checkCollisionForX(float x) {
    const float leftWall = -kArenaHalfWidth;
    const float rightWall = kArenaHalfWidth;
    const float leftEdge = x - kMarkerRadius;
    const float rightEdge = x + kMarkerRadius;
    return (rightEdge >= rightWall || leftEdge <= leftWall) ? -1 : 1;
}

int checkCollisionForY(float y) {
    const float topWall = kArenaHalfHeight;
    const float bottomWall = -kArenaHalfHeight;
    const float topEdge = y + kMarkerRadius;
    const float bottomEdge = y - kMarkerRadius;
    return (topEdge >= topWall || bottomEdge <= bottomWall) ? -1 : 1;
}

float correctXPosition(float x) {
    const float leftWall = -kArenaHalfWidth;
    const float rightWall = kArenaHalfWidth;
    const float leftEdge = x - kMarkerRadius;
    const float rightEdge = x + kMarkerRadius;
    if (leftEdge < leftWall) {
        return x + (leftWall - leftEdge);
    }
    if (rightEdge > rightWall) {
        return x - (rightEdge - rightWall);
    }
    return x;
}

float correctYPosition(float y) {
    const float topWall = kArenaHalfHeight;
    const float bottomWall = -kArenaHalfHeight;
    const float topEdge = y + kMarkerRadius;
    const float bottomEdge = y - kMarkerRadius;
    if (topEdge > topWall) {
        return y - (topEdge - topWall);
    }
    if (bottomEdge < bottomWall) {
        return y + (bottomWall - bottomEdge);
    }
    return y;
}

void updateMarker(float dt) {
    g_position.x += g_velocity.x * dt;
    g_position.y += g_velocity.y * dt;

    if (checkCollisionForX(g_position.x) == -1) {
        g_position.x = correctXPosition(g_position.x);
        g_velocity.x *= -1.0f;
    }
    if (checkCollisionForY(g_position.y) == -1) {
        g_position.y = correctYPosition(g_position.y);
        g_velocity.y *= -1.0f;
    }
}

void appendCircleXZ(std::vector<Vec3>& outVerts, float cx, float cz, float radius, int segments) {
    for (int i = 0; i < segments; ++i) {
        const float t = (static_cast<float>(i) / static_cast<float>(segments)) * 6.283185307f;
        outVerts.push_back({cx + std::cos(t) * radius, 0.0f, cz + std::sin(t) * radius});
    }
}

void appendGrid(std::vector<Vec3>& outVerts, float halfW, float halfH, float spacing) {
    for (float x = -halfW; x <= halfW + 0.01f; x += spacing) {
        outVerts.push_back({x, 0.0f, -halfH});
        outVerts.push_back({x, 0.0f, halfH});
    }
    for (float z = -halfH; z <= halfH + 0.01f; z += spacing) {
        outVerts.push_back({-halfW, 0.0f, z});
        outVerts.push_back({halfW, 0.0f, z});
    }
}

void drawLineBatch(const std::vector<Vec3>& vertices, GLenum primitive, const Vec3& color, const Mat4& mvp) {
    if (vertices.empty()) {
        return;
    }

    glUseProgram(g_shaderProgram);
    glUniformMatrix4fv(g_uMvpLoc, 1, GL_FALSE, mvp.m);
    glUniform3f(g_uColorLoc, color.x, color.y, color.z);
    glBindVertexArray(g_lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, g_lineVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vec3)), vertices.data());
    glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
    glBindVertexArray(0);
}

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

    g_shaderProgram = createProgram(kVertSrc, kFragSrc);
    if (g_shaderProgram == 0) {
        glfwTerminate();
        return -1;
    }
    g_uMvpLoc = glGetUniformLocation(g_shaderProgram, "uMVP");
    g_uColorLoc = glGetUniformLocation(g_shaderProgram, "uColor");

    glGenVertexArrays(1, &g_lineVao);
    glGenBuffers(1, &g_lineVbo);
    glBindVertexArray(g_lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, g_lineVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(kMaxDebugVertices * sizeof(Vec3)), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.5f);

    initFrameTimer();

    std::vector<Vec3> gridVerts;
    gridVerts.reserve(2000);
    appendGrid(gridVerts, kArenaHalfWidth, kArenaHalfHeight, 50.0f);

    while (!glfwWindowShouldClose(window)) {
        const float dt = computeDeltaTimeSeconds();
        processInput(window, dt);
        updateCameraMouseOrbit(window);
        g_cameraPitch = kIsometricPitchRadians; // Enforce isometric view every frame.
        updateMarker(dt);

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

        drawLineBatch(gridVerts, GL_LINES, {0.23f, 0.24f, 0.28f}, mvp);

        const std::vector<Vec3> axisX = {{0.0f, 0.0f, 0.0f}, {220.0f, 0.0f, 0.0f}};
        const std::vector<Vec3> axisY = {{0.0f, 0.0f, 0.0f}, {0.0f, 220.0f, 0.0f}};
        const std::vector<Vec3> axisZ = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 220.0f}};
        drawLineBatch(axisX, GL_LINES, {1.0f, 0.2f, 0.2f}, mvp);
        drawLineBatch(axisY, GL_LINES, {0.2f, 1.0f, 0.2f}, mvp);
        drawLineBatch(axisZ, GL_LINES, {0.2f, 0.6f, 1.0f}, mvp);

        const std::vector<Vec3> bounds = {
            {-kArenaHalfWidth, 0.0f, -kArenaHalfHeight},
            { kArenaHalfWidth, 0.0f, -kArenaHalfHeight},
            { kArenaHalfWidth, 0.0f,  kArenaHalfHeight},
            {-kArenaHalfWidth, 0.0f,  kArenaHalfHeight}
        };
        drawLineBatch(bounds, GL_LINE_LOOP, {0.8f, 0.8f, 0.9f}, mvp);

        std::vector<Vec3> marker;
        marker.reserve(kCircleSegments);
        appendCircleXZ(marker, g_position.x, g_position.y, kMarkerRadius, kCircleSegments);
        drawLineBatch(marker, GL_LINE_LOOP, {1.0f, 0.9f, 0.25f}, mvp);

        const Vec3 markerCenter = {g_position.x, 0.0f, g_position.y};
        const Vec3 velDir = normalize({g_velocity.x, 0.0f, g_velocity.y});
        const std::vector<Vec3> velLine = {markerCenter, markerCenter + velDir * (kMarkerRadius * 2.2f)};
        drawLineBatch(velLine, GL_LINES, {1.0f, 0.5f, 0.1f}, mvp);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteBuffers(1, &g_lineVbo);
    glDeleteVertexArrays(1, &g_lineVao);
    glDeleteProgram(g_shaderProgram);
    glfwTerminate();
    return 0;
}
