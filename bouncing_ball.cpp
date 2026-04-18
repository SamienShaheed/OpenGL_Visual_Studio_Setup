/*
TO-DO: (Does not handle acceleration)
    - Remove hardcoded (0,0) coords for circle center
    - Use Pos vector instead (x, y)
    - Define velocity (vx, vy)
    - Compute deltatime each frame
    - Add velocity to Pos times deltatime ([x += vx]*deltatime, [y += vy]*deltatime)
    - Check for collisions with the window 
    - If (Pos + Radius) hits wall, bounce (-x for vert wall, -y for horz wall)
*/

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std;

// create struct to hold 2D vectors
struct Vec2 {
    float x, y;
};

// Global parameters
int WINDOW_WIDTH = 1560;
int WINDOW_HEIGHT = 840;
int HALF_WIDTH = (int)(WINDOW_WIDTH/2);
int HALF_HEIGHT = (int)(WINDOW_HEIGHT/2);
int RADIUS = 50;
int SEGMENTS = 100;
std::string WINDOW_TITLE = "Physics Test";

Vec2 VELOCITY = {-1000.0f, -1000.0f};
Vec2 position = {0, 0};

double lastFrameTime = glfwGetTime();

// Initialize the window
void myInit (void) {
    glClearColor(0.0, 0.0, 0.0, 1.0); // rgba
    glColor3f(1.0, 1.0, 1.0); // rgb
    glPointSize(1.0); // point size
    glMatrixMode(GL_PROJECTION); // projection matrix
    glLoadIdentity(); // load identity matrix

    glOrtho(-HALF_WIDTH, HALF_WIDTH, -HALF_HEIGHT, HALF_HEIGHT, -1, 1); // orthographic projection
}

// Draw the circle
void drawCircle(float center_x, float center_y, float radius, int num_segments) {
    float angle_step = 3.1415926 * 2 / float(num_segments);
    float tangent_length = tanf(angle_step);
    float radius_correction = cosf(angle_step);

    float current_x = radius;
    float current_y = 0;
    
    glLineWidth(2);
    glBegin(GL_LINE_LOOP);
    for (int segment = 0; segment < num_segments; segment++)
    {
        glVertex2f(current_x + center_x, current_y + center_y);

        float tangent_x = -current_y;
        float tangent_y = current_x;

        current_x += tangent_x * tangent_length;
        current_y += tangent_y * tangent_length;

        current_x *= radius_correction;
        current_y *= radius_correction;
    }
    glEnd();
}

// find deltatime
double computeDeltatime() {
    double currentTime = glfwGetTime();
    double deltaTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime; // update last frame time
    return deltaTime;
}

// check if collides with wall, if yes return -1 otherwise returns 1
int checkCollisionForX(float x) {
    // wall coordinates
    float leftWall = -HALF_WIDTH;
    float rightWall = HALF_WIDTH;
    // ball coordinates
    float leftEdge = x - RADIUS;
    float rightEdge = x + RADIUS;

    if (rightEdge >= rightWall || leftEdge <= leftWall) {
        return -1;
    }

    return 1;
}

// check if collides with wall, if yes return -1 otherwise returns 1
int checkCollisionForY(float y) {
    // wall coordinates
    float topWall = HALF_HEIGHT;
    float bottomWall = -HALF_HEIGHT;
    // ball coordinates
    float topEdge = y + RADIUS;
    float bottomEdge = y - RADIUS;

    if (topEdge >= topWall || bottomEdge <= bottomWall) {
        return -1;
    }

    return 1;
}

// correct X Position on collision
float correctXPosition(float x) {
    float leftWall  = -HALF_WIDTH;
    float rightWall =  HALF_WIDTH;
    float leftEdge  = x - RADIUS;
    float rightEdge = x + RADIUS;

    if (leftEdge < leftWall) {
        float overlap = leftWall - leftEdge;
        return x + overlap; // move ball right
    }
    if (rightEdge > rightWall) {
        float overlap = rightEdge - rightWall;
        return x - overlap; // move ball left
    }
    return x;
}

// correct Y Position on collision
float correctYPosition(float y) {
    float topWall    =  HALF_HEIGHT;
    float bottomWall = -HALF_HEIGHT;
    float topEdge    = y + RADIUS;
    float bottomEdge = y - RADIUS;

    if (topEdge > topWall) {
        float overlap = topEdge - topWall;
        return y - overlap; // move ball down
    }
    if (bottomEdge < bottomWall) {
        float overlap = bottomWall - bottomEdge;
        return y + overlap; // move ball up
    }
    return y;
}

// Main function
int main() {
    glfwInit();
    
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE.c_str(), NULL, NULL); // generate window pointer
    glfwMakeContextCurrent(window);
    
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    
    myInit();

    double deltatime;
    float x = position.x;
    float y = position.y;

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);
        
        drawCircle(x, y, RADIUS, SEGMENTS);

        if (checkCollisionForX(x) == -1) { x = correctXPosition(x);}
        if (checkCollisionForY(y) == -1) { y = correctYPosition(y);}

        // update velocity based on collision
        VELOCITY.x = VELOCITY.x * checkCollisionForX(x);
        VELOCITY.y = VELOCITY.y * checkCollisionForY(y);

        // change position vector based on velocity
        deltatime = computeDeltatime();
        x += (VELOCITY.x) * deltatime;
        y += (VELOCITY.y) * deltatime;

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwTerminate();
    return 0;
}