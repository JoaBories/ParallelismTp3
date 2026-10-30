#include "boids.hpp"
#include "gl_utils.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool nearly_equal(const BoidSim& a, const BoidSim& b, float eps = 1e-4f) {
  if (a.boids().size() != b.boids().size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.boids().size(); ++i) {
    const float dx = a.boids()[i].pos.x - b.boids()[i].pos.x;
    const float dy = a.boids()[i].pos.y - b.boids()[i].pos.y;
    if (dx * dx + dy * dy > eps * eps) {
      return false;
    }
  }
  return true;
}

bool run_check() {
  BoidSim sim(4096, 7);
  bool assign_ok = true;
  for (int i = 0; i < 50; ++i) {
    sim.step(sim.params().dt);
    if (!sim.grid_assignment_ok()) {
      assign_ok = false;
      break;
    }
  }

  BoidSim parallel(2048, 42);
  BoidSim reference(2048, 42);
  for (int i = 0; i < 15; ++i) {
    parallel.step(parallel.params().dt);
    reference.step_sequential_reference(reference.params().dt);
  }
  const bool same = nearly_equal(parallel, reference);

  const bool ok = assign_ok && same;
  std::cout << (assign_ok ? "[OK]   " : "[FAIL] ")
            << "chaque boid dans exactement une cellule\n";
  std::cout << (same ? "[OK]   " : "[FAIL] ")
            << "pas parallele = reference sequentielle\n";
  return ok;
}

void run_window(int boid_count) {
  GLFWwindow* window = initWindow(1600, 1600, "TP2 02 — affectation parallèle");
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
    std::snprintf(title, sizeof(title), "TP2 02 — affectation | step %.2f ms", ms);
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
