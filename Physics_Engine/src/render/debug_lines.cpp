#include "render/debug_lines.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

GLuint g_shaderProgram = 0;
GLint g_uMvpLoc = -1;
GLint g_uColorLoc = -1;
GLuint g_lineVao = 0;
GLuint g_lineVbo = 0;
constexpr int kMaxDebugVertices = 12000;

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

} // namespace

void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
    glViewport(0, 0, width, height);
}

bool initDebugLineRenderer() {
    g_shaderProgram = createProgram(kVertSrc, kFragSrc);
    if (g_shaderProgram == 0) {
        return false;
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
    return true;
}

void shutdownDebugLineRenderer() {
    glDeleteBuffers(1, &g_lineVbo);
    glDeleteVertexArrays(1, &g_lineVao);
    glDeleteProgram(g_shaderProgram);
    g_lineVbo = 0;
    g_lineVao = 0;
    g_shaderProgram = 0;
}

void appendCircleXZAtY(std::vector<Vec3>& outVerts, float cx, float cy, float cz, float radius, int segments) {
    for (int i = 0; i < segments; ++i) {
        const float t = (static_cast<float>(i) / static_cast<float>(segments)) * 6.283185307f;
        outVerts.push_back({cx + std::cos(t) * radius, cy, cz + std::sin(t) * radius});
    }
}

void appendArenaGrid(std::vector<Vec3>& outVerts, float halfW, float halfD, float y, int divisions) {
    for (int i = 0; i <= divisions; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(divisions);
        const float x = -halfW + 2.0f * halfW * t;
        const float z = -halfD + 2.0f * halfD * t;
        outVerts.push_back({x, y, -halfD});
        outVerts.push_back({x, y, halfD});
        outVerts.push_back({-halfW, y, z});
        outVerts.push_back({halfW, y, z});
    }
}

void appendArenaBorder(std::vector<Vec3>& outVerts, float halfW, float halfD, float y) {
    const std::vector<Vec3> corners = {
        {-halfW, y, -halfD},
        {halfW, y, -halfD},
        {halfW, y, halfD},
        {-halfW, y, halfD}
    };
    for (int i = 0; i < 4; ++i) {
        outVerts.push_back(corners[i]);
        outVerts.push_back(corners[(i + 1) % 4]);
    }
}

void drawLineBatch(const std::vector<Vec3>& vertices, unsigned int primitive, const Vec3& color, const Mat4& mvp) {
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

void drawWorldAxesGizmo(const Mat4& view, int fbw, int fbh) {
    if (fbw <= 0 || fbh <= 0) {
        return;
    }

    const int gizmoSize = std::clamp(std::min(fbw, fbh) / 5, 72, 160);
    const int gx = 0;
    const int gy = fbh - gizmoSize;

    Vec3 dx = mat4TransformDirection(view, {1.0f, 0.0f, 0.0f});
    Vec3 dy = mat4TransformDirection(view, {0.0f, 1.0f, 0.0f});
    Vec3 dz = mat4TransformDirection(view, {0.0f, 0.0f, 1.0f});
    dx = normalize(dx);
    dy = normalize(dy);
    dz = normalize(dz);

    const float len = 0.82f;
    const std::vector<Vec3> lineX = {{0.0f, 0.0f, 0.0f}, dx * len};
    const std::vector<Vec3> lineY = {{0.0f, 0.0f, 0.0f}, dy * len};
    const std::vector<Vec3> lineZ = {{0.0f, 0.0f, 0.0f}, dz * len};

    const Mat4 ortho = mat4Ortho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

    glEnable(GL_SCISSOR_TEST);
    glViewport(gx, gy, gizmoSize, gizmoSize);
    glScissor(gx, gy, gizmoSize, gizmoSize);
    glClear(GL_DEPTH_BUFFER_BIT);

    drawLineBatch(lineX, GL_LINES, {0.92f, 0.28f, 0.28f}, ortho);
    drawLineBatch(lineY, GL_LINES, {0.32f, 0.92f, 0.35f}, ortho);
    drawLineBatch(lineZ, GL_LINES, {0.28f, 0.5f, 1.0f}, ortho);

    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, fbw, fbh);
}
