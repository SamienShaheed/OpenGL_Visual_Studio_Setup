#include "render/debug_lines.hpp"

#include "math/math.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

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

void drawCircleBodyXZPlane(
    const Vec3& comWorld,
    const Quaternion& bodyOrientation,
    float yBody,
    float ringRadius,
    int segments,
    const Vec3& color,
    const Mat4& mvp) {
    if (ringRadius <= 0.0f || segments < 3) {
        return;
    }
    const Vec3 c = comWorld + quatRotateVector(bodyOrientation, {0.0f, yBody, 0.0f});
    const Vec3 bx = quatRotateVector(bodyOrientation, {1.0f, 0.0f, 0.0f});
    const Vec3 bz = quatRotateVector(bodyOrientation, {0.0f, 0.0f, 1.0f});
    const float kTwoPi = 6.283185307f;
    std::vector<Vec3> loop;
    loop.reserve(static_cast<std::size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        const float t = (static_cast<float>(i) / static_cast<float>(segments)) * kTwoPi;
        loop.push_back(c + bx * (std::cos(t) * ringRadius) + bz * (std::sin(t) * ringRadius));
    }
    drawLineBatch(loop, GL_LINE_LOOP, color, mvp);
}

void drawWireframeSphereWorldAxes(const Vec3& c, float radius, int segments, const Vec3& color, const Mat4& mvp) {
    if (radius <= 0.0f || segments < 3) {
        return;
    }
    const float kTwoPi = 6.283185307f;

    std::vector<Vec3> xz;
    xz.reserve(static_cast<std::size_t>(segments));
    appendCircleXZAtY(xz, c.x, c.y, c.z, radius, segments);
    drawLineBatch(xz, GL_LINE_LOOP, color, mvp);

    std::vector<Vec3> xy;
    xy.reserve(static_cast<std::size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        const float t = (static_cast<float>(i) / static_cast<float>(segments)) * kTwoPi;
        xy.push_back({c.x + std::cos(t) * radius, c.y + std::sin(t) * radius, c.z});
    }
    drawLineBatch(xy, GL_LINE_LOOP, color, mvp);

    std::vector<Vec3> yz;
    yz.reserve(static_cast<std::size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        const float t = (static_cast<float>(i) / static_cast<float>(segments)) * kTwoPi;
        yz.push_back({c.x, c.y + std::cos(t) * radius, c.z + std::sin(t) * radius});
    }
    drawLineBatch(yz, GL_LINE_LOOP, color, mvp);
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

void appendArenaWallFrame(std::vector<Vec3>& outVerts, float halfW, float halfD, float floorY, float wallHeight) {
    const float y0 = floorY;
    const float y1 = floorY + wallHeight;
    const std::vector<Vec3> corners = {
        {-halfW, y0, -halfD},
        {halfW, y0, -halfD},
        {halfW, y0, halfD},
        {-halfW, y0, halfD}
    };
    for (const Vec3& c : corners) {
        outVerts.push_back(c);
        outVerts.push_back({c.x, y1, c.z});
    }
    appendArenaBorder(outVerts, halfW, halfD, y1);
}

void appendBowlParaboloidWireframe(
    std::vector<Vec3>& outVerts, float y0, float k, float maxR, int meridians, int radialSegments) {
    if (maxR <= 0.0f || meridians < 3 || radialSegments < 2) {
        return;
    }
    const float kClamped = std::max(0.0f, k);
    const int m = meridians;
    const int rs = radialSegments;
    const float kTwoPi = 6.283185307f;
    for (int mi = 0; mi < m; ++mi) {
        const float ang = (static_cast<float>(mi) / static_cast<float>(m)) * kTwoPi;
        const float ca = std::cos(ang);
        const float sa = std::sin(ang);
        for (int j = 0; j < rs; ++j) {
            const float t0 = static_cast<float>(j) / static_cast<float>(rs);
            const float t1 = static_cast<float>(j + 1) / static_cast<float>(rs);
            const float r0 = maxR * t0;
            const float r1 = maxR * t1;
            const float yA = y0 + kClamped * (r0 * r0);
            const float yB = y0 + kClamped * (r1 * r1);
            outVerts.push_back({ca * r0, yA, sa * r0});
            outVerts.push_back({ca * r1, yB, sa * r1});
        }
    }
    for (int j = 1; j <= rs; ++j) {
        const float r = maxR * (static_cast<float>(j) / static_cast<float>(rs));
        const float y = y0 + kClamped * (r * r);
        for (int mi = 0; mi < m; ++mi) {
            const float ang0 = (static_cast<float>(mi) / static_cast<float>(m)) * kTwoPi;
            const float ang1 = (static_cast<float>(mi + 1) / static_cast<float>(m)) * kTwoPi;
            outVerts.push_back({std::cos(ang0) * r, y, std::sin(ang0) * r});
            outVerts.push_back({std::cos(ang1) * r, y, std::sin(ang1) * r});
        }
    }
}

void appendBowlRimWallFrame(
    std::vector<Vec3>& outVerts, float y0, float k, float maxR, float wallHeight, int segments) {
    if (maxR <= 0.0f || wallHeight <= 0.0f || segments < 3) {
        return;
    }
    const float kClamped = std::max(0.0f, k);
    const float yRim = y0 + kClamped * maxR * maxR;
    const float yTop = yRim + wallHeight;
    const float kTwoPi = 6.283185307f;
    for (int i = 0; i < segments; ++i) {
        const float t0 = (static_cast<float>(i) / static_cast<float>(segments)) * kTwoPi;
        const float t1 = (static_cast<float>(i + 1) / static_cast<float>(segments)) * kTwoPi;
        const float x0 = std::cos(t0) * maxR;
        const float z0 = std::sin(t0) * maxR;
        const float x1 = std::cos(t1) * maxR;
        const float z1 = std::sin(t1) * maxR;
        outVerts.push_back({x0, yRim, z0});
        outVerts.push_back({x0, yTop, z0});
        outVerts.push_back({x0, yTop, z0});
        outVerts.push_back({x1, yTop, z1});
    }
    for (int i = 0; i < segments; ++i) {
        const float t0 = (static_cast<float>(i) / static_cast<float>(segments)) * kTwoPi;
        const float t1 = (static_cast<float>(i + 1) / static_cast<float>(segments)) * kTwoPi;
        const float x0 = std::cos(t0) * maxR;
        const float z0 = std::sin(t0) * maxR;
        const float x1 = std::cos(t1) * maxR;
        const float z1 = std::sin(t1) * maxR;
        outVerts.push_back({x0, yRim, z0});
        outVerts.push_back({x1, yRim, z1});
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

void drawVelocityGuides(const Vec3& com, const Vec3& v, float topRadius, const Mat4& mvp) {
    const float speed = length(v);
    constexpr float kHideBelow = 0.32f;
    if (speed < kHideBelow) {
        return;
    }

    const Vec3 dir = v * (1.0f / speed);
    const float mainLen =
        std::clamp(topRadius * 0.88f + speed * 0.044f, topRadius * 0.75f, topRadius * 5.0f);
    const std::vector<Vec3> totalVel = {com, com + dir * mainLen};
    drawLineBatch(totalVel, GL_LINES, {1.0f, 0.45f, 0.1f}, mvp);

    const Vec3 vxz = {v.x, 0.0f, v.z};
    const float horizSpeed = length(vxz);
    // Gold line = horizontal (XZ) part only at COM height. Skip when motion is already horizontal (would
    // duplicate the orange arrow) or when vertical motion is negligible.
    if (horizSpeed >= 0.26f && std::fabs(v.y) > 0.14f) {
        const Vec3 hdir = vxz * (1.0f / horizSpeed);
        const float hLen =
            std::clamp(topRadius * 0.48f + horizSpeed * 0.042f, topRadius * 0.4f, topRadius * 4.8f);
        const Vec3 slideEnd = com + hdir * hLen;
        const std::vector<Vec3> slideLine = {com, slideEnd};
        drawLineBatch(slideLine, GL_LINES, {0.98f, 0.78f, 0.28f}, mvp);
    }

    const float vy = v.y;
    if (std::fabs(vy) >= 0.22f) {
        const float vyLen =
            std::clamp(std::fabs(vy) * 0.16f, topRadius * 0.15f, topRadius * 1.75f) * (vy >= 0.0f ? 1.0f : -1.0f);
        const Vec3 vyEnd = {com.x, com.y + vyLen, com.z};
        const std::vector<Vec3> vertTick = {com, vyEnd};
        drawLineBatch(vertTick, GL_LINES, {0.35f, 0.8f, 1.0f}, mvp);
    }
}
