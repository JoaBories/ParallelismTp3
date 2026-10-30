#pragma once

#include <cstdint>
#include <thread>
#include <vector>

struct Vec2 {
  float x = 0.f;
  float y = 0.f;
};

struct Boid {
  Vec2 pos;
  Vec2 vel;
  Vec2 heading;
};

struct SimParams {
  float box = 128.f;
  int grid_n = 128;
  float r_sep = 0.5f;
  float r_ali = 1.0f;
  float r_coh = 1.0f;
  float w_sep = 1.6f;
  float w_ali = 1.0f;
  float w_coh = 1.0f;
  float max_speed = 4.0f;
  float max_force = 8.0f;
  float dt = 1.f / 60.f;
};

class BoidSim {
 public:
  explicit BoidSim(int count, std::uint32_t seed = 1);

  void step(float dt);
  void step_sequential_reference(float dt);

  const std::vector<Boid>& boids() const { return boids_; }
  const SimParams& params() const { return params_; }
  void pack_positions(std::vector<float>& out) const;
  bool grid_assignment_ok() const;

  double last_grid_ms() const { return last_grid_ms_; }
  double last_forces_ms() const { return last_forces_ms_; }

 private:
  static int default_worker_count() {
    unsigned cores = std::thread::hardware_concurrency();
    if (cores == 0) {
      cores = 1;
    }
    return static_cast<int>(cores * 2);
  }

  void rebuild_grid();
  void compute_forces_range(int begin, int end, std::vector<Vec2>& forces) const;
  void compute_forces_parallel(std::vector<Vec2>& forces) const;
  void integrate(const std::vector<Vec2>& forces, float dt);

  int cell_index(const Vec2& p) const;
  void neighbors(int cell, std::vector<int>& out) const;

  SimParams params_;
  std::vector<Boid> boids_;
  std::vector<std::vector<int>> cells_;
  int worker_count_ = default_worker_count();
  double last_grid_ms_ = 0.0;
  double last_forces_ms_ = 0.0;
};
