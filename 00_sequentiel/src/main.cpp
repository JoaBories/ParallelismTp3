#include "boids.hpp"
#include "gl_utils.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool run_check() {
  BoidSim sim(2048, 42);
  for (int i = 0; i < 30; ++i) {
    sim.step(sim.params().dt);
  }
  if (!sim.grid_assignment_ok()) {
    std::cerr << "[FAIL] affectation grille\n";
    return false;
  }
  double checksum = 0.0;
  for (const auto& b : sim.boids()) {
    checksum += b.pos.x + 0.1 * b.pos.y + 0.01 * b.vel.x;
  }
  std::cout << "[OK]   check séquentiel checksum=" << checksum << '\n';
  return true;
}

void run_window(int boid_count) {
  GLFWwindow* window = initWindow(1600, 1600, "TP2 00 — boids séquentiels");
  if (!window) {
    std::cerr << "GLFW init failed\n";
    return;
  }
  
  glewExperimental = GL_TRUE;

  if (glewInit() != GLEW_OK) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return;
  }

  printf("OpenGL : %s\n", glGetString(GL_VERSION));

  BoidSim sim(boid_count, 1);
  std::vector<float> positions;
  auto last = Clock::now();
  double grid_sum_ms = 0.0;
  double forces_sum_ms = 0.0;
  int step_count = 0;
  auto report = last;

  while (!windowShouldClose()) {
    const auto now = Clock::now();
    float dt = std::chrono::duration<float>(now - last).count();
    last = now;
    if (dt > 0.05f) {
      dt = 0.05f;
    }

    sim.step(dt);
    const double grid_ms = sim.last_grid_ms();
    const double forces_ms = sim.last_forces_ms();
    const double ms = grid_ms + forces_ms;
    grid_sum_ms += grid_ms;
    forces_sum_ms += forces_ms;
    ++step_count;
    if (std::chrono::duration<double>(Clock::now() - report).count() >= 1.0) {
      std::printf("grille %.2f ms | forces %.2f ms (%d frames)\n",
                  grid_sum_ms / step_count, forces_sum_ms / step_count, step_count);
      std::fflush(stdout);
      grid_sum_ms = 0.0;
      forces_sum_ms = 0.0;
      step_count = 0;
      report = Clock::now();
    }

    char title[128];
    std::snprintf(title, sizeof(title), "TP2 00 — séquentiel | step %.2f ms", ms);
    setWindowTitle(title);

    sim.pack_positions(positions);
    drawPositions(positions.data(), static_cast<int>(sim.boids().size()), sim.params().box);
    pollAndSwap();
  }

  shutdownWindow();
}

}  // namespace

int main(int argc, char** argv) {
  int boid_count = 80000;
  bool check = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--check") {
      check = true;
    } else if (arg == "--c" && i + 1 < argc) {
      boid_count = std::stoi(argv[++i]);
    }
  }
  if (check) {
    return run_check() ? 0 : 1;
  }
  run_window(boid_count);
  return 0;
}
