#include "gl_utils.hpp"

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

GLFWwindow* gWindow = nullptr;
int gWidth = 800;
int gHeight = 800;
GLuint gProgram = 0;
GLuint gVao = 0;
GLuint gVbo = 0;

std::string readFile(const std::string& path) {
  std::ifstream in(path.c_str());
  if (!in) {
    throw std::runtime_error("lecture impossible : " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

GLuint compile(GLenum type, const std::string& src, const std::string& name) {
  GLuint shader = glCreateShader(type);
  const char* c = src.c_str();
  glShaderSource(shader, 1, &c, nullptr);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    throw std::runtime_error(name + " : " + log);
  }
  return shader;
}

GLuint link(GLuint vert, GLuint geom, GLuint frag) {
  GLuint program = glCreateProgram();
  glAttachShader(program, vert);
  glAttachShader(program, geom);
  glAttachShader(program, frag);
  glLinkProgram(program);
  GLint ok = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetProgramInfoLog(program, sizeof(log), nullptr, log);
    throw std::runtime_error(std::string("link : ") + log);
  }
  glDeleteShader(vert);
  glDeleteShader(geom);
  glDeleteShader(frag);
  return program;
}

std::string shaderPath(const char* path) {
  std::ifstream direct(path);
  if (direct) {
    return path;
  }
  return std::string("shaders") + "/" + path;
}

void ensureProgram() {
  if (gProgram) {
    return;
  }
  gProgram = link(compile(GL_VERTEX_SHADER, readFile(shaderPath("points.vert")), "points.vert"),
                  compile(GL_GEOMETRY_SHADER, readFile(shaderPath("points.geom")), "points.geom"),
                  compile(GL_FRAGMENT_SHADER, readFile(shaderPath("points.frag")), "points.frag"));
  glGenVertexArrays(1, &gVao);
  glGenBuffers(1, &gVbo);
}

}  // namespace

GLFWwindow* initWindow(int width, int height, const char* title) {
  gWidth = width;
  gHeight = height;
  if (!glfwInit()) {
    return nullptr;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  gWindow = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!gWindow) {
    glfwTerminate();
    return nullptr;
  }
  glfwMakeContextCurrent(gWindow);
  glfwSwapInterval(1);
  return gWindow;
}

bool windowShouldClose() { return glfwWindowShouldClose(gWindow) != 0; }

void pollAndSwap() {
  glfwSwapBuffers(gWindow);
  glfwPollEvents();
  int w = 0;
  int h = 0;
  glfwGetFramebufferSize(gWindow, &w, &h);
  if (w > 0 && h > 0) {
    gWidth = w;
    gHeight = h;
  }
}

void shutdownWindow() {
  if (gVbo) {
    glDeleteBuffers(1, &gVbo);
    gVbo = 0;
  }
  if (gVao) {
    glDeleteVertexArrays(1, &gVao);
    gVao = 0;
  }
  if (gProgram) {
    glDeleteProgram(gProgram);
    gProgram = 0;
  }
  if (gWindow) {
    glfwDestroyWindow(gWindow);
    gWindow = nullptr;
  }
  glfwTerminate();
}

void setWindowTitle(const char* title) {
  if (gWindow) {
    glfwSetWindowTitle(gWindow, title);
  }
}

void drawPositions(const float* xy, int count, float box_size) {
  ensureProgram();
  glViewport(0, 0, gWidth, gHeight);
  glDisable(GL_DEPTH_TEST);
  glClearColor(0.05f, 0.07f, 0.1f, 1.f);
  glClear(GL_COLOR_BUFFER_BIT);

  const float half = box_size * 0.5f;
  glm::mat4 view(1.f);
  glm::mat4 proj = glm::ortho(-half, half, -half, half, -1.f, 1.f);

  glUseProgram(gProgram);
  glUniformMatrix4fv(glGetUniformLocation(gProgram, "uView"), 1, GL_FALSE, &view[0][0]);
  glUniformMatrix4fv(glGetUniformLocation(gProgram, "uProj"), 1, GL_FALSE, &proj[0][0]);

  glBindVertexArray(gVao);
  glBindBuffer(GL_ARRAY_BUFFER, gVbo);
  const GLsizei stride = 4 * sizeof(float);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(count) * stride, xy, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(float)));
  glDrawArrays(GL_POINTS, 0, count);
  glBindVertexArray(0);
}
