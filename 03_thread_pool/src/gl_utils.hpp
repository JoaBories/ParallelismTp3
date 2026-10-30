#pragma once
#include "gl/glew.h"
#include "GLFW/glfw3.h"
#include "glm/mat4x4.hpp"

GLFWwindow* initWindow(int width, int height, const char* title);
bool windowShouldClose();
void pollAndSwap();
void shutdownWindow();
void setWindowTitle(const char* title);
void drawPositions(const float* xy, int count, float box_size);
