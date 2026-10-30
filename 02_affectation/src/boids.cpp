#include "boids.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

float length(Vec2 v) { return std::sqrt(v.x * v.x + v.y * v.y); }

Vec2 normalize_or_zero(Vec2 v) {
  const float L = length(v);
  if (L < 1e-6f) {
    return {};
  }
  return {v.x / L, v.y / L};
}

Vec2 clamp_length(Vec2 v, float max_len) {
  const float L = length(v);
  if (L > max_len && L > 1e-6f) {
    const float s = max_len / L;
    return {v.x * s, v.y * s};
  }
  return v;
}

Vec2 wrap(Vec2 p, float box) {
  const float half = box * 0.5f;
  while (p.x < -half) {
    p.x += box;
  }
  while (p.x >= half) {
    p.x -= box;
  }
  while (p.y < -half) {
    p.y += box;
  }
  while (p.y >= half) {
    p.y -= box;
  }
  return p;
}

float toroidal_delta(float a, float b, float box) {
  float d = a - b;
  const float half = box * 0.5f;
  if (d > half) {
    d -= box;
  }
  if (d < -half) {
    d += box;
  }
  return d;
}


void smooth_heading(Vec2& heading, Vec2 vel, float dt) {
  const Vec2 dir = normalize_or_zero(vel);
  if (dir.x == 0.f && dir.y == 0.f) {
    return;
  }
  if (heading.x == 0.f && heading.y == 0.f) {
    heading = dir;
    return;
  }
  const float k = 1.f - std::exp(-4.f * dt);
  heading.x += (dir.x - heading.x) * k;
  heading.y += (dir.y - heading.y) * k;
  heading = normalize_or_zero(heading);
  if (heading.x == 0.f && heading.y == 0.f) {
    heading = dir;
  }
}

}  // namespace

BoidSim::BoidSim(int count, std::uint32_t seed) {
  const int ncells = params_.grid_n * params_.grid_n;
  const std::size_t cell_count = static_cast<std::size_t>(ncells);
  // Capacité par cellule ~ 4× la densité moyenne (évite le trop-plein local).
  cell_cap_ = std::max(64, (4 * count + ncells - 1) / ncells);
  cell_counts_.assign(cell_count, 0);
  cell_ids_.assign(cell_count * static_cast<std::size_t>(cell_cap_), 0);
  boids_.resize(static_cast<std::size_t>(count));
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> pos(-params_.box * 0.45f, params_.box * 0.45f);
  std::uniform_real_distribution<float> ang(0.f, 6.2831853f);
  for (auto& b : boids_) {
    b.pos = {pos(rng), pos(rng)};
    const float a = ang(rng);
    b.vel = {std::cos(a) * params_.max_speed * 0.5f, std::sin(a) * params_.max_speed * 0.5f};
    b.heading = normalize_or_zero(b.vel);
  }
}

int BoidSim::cell_index(const Vec2& p) const {
  const float half = params_.box * 0.5f;
  const float cell = params_.box / static_cast<float>(params_.grid_n);
  int cx = static_cast<int>((p.x + half) / cell);
  int cy = static_cast<int>((p.y + half) / cell);
  cx = std::clamp(cx, 0, params_.grid_n - 1);
  cy = std::clamp(cy, 0, params_.grid_n - 1);
  return cy * params_.grid_n + cx;
}

void BoidSim::neighbors(int cell, std::vector<int>& out) const {
  out.clear();
  const int cx = cell % params_.grid_n;
  const int cy = cell / params_.grid_n;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      int nx = cx + dx;
      int ny = cy + dy;
      if (nx < 0) {
        nx += params_.grid_n;
      }
      if (nx >= params_.grid_n) {
        nx -= params_.grid_n;
      }
      if (ny < 0) {
        ny += params_.grid_n;
      }
      if (ny >= params_.grid_n) {
        ny -= params_.grid_n;
      }
      out.push_back(ny * params_.grid_n + nx);
    }
  }
}

void BoidSim::rebuild_grid_sequential() {
  std::fill(cell_counts_.begin(), cell_counts_.end(), 0);
  for (int i = 0; i < static_cast<int>(boids_.size()); ++i) {
    const int c = cell_index(boids_[static_cast<std::size_t>(i)].pos);
    const int prev = cell_counts_[static_cast<std::size_t>(c)]++;
    if (prev < cell_cap_) {
      cell_ids_[static_cast<std::size_t>(c * cell_cap_ + prev)] = i;
    }
  }
}

void BoidSim::rebuild_grid() {
  std::fill(cell_counts_.begin(), cell_counts_.end(), 0);
  const int n = static_cast<int>(boids_.size());
  const int w = std::max(1, worker_count_);
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(w));

  for (int t = 0; t < w; ++t) {
    const int begin = n * t / w;
    const int end = std::min(n * (t + 1) / w, n);
    workers.emplace_back([this, begin, end] {
      for (int i = begin; i < end; ++i) {
        const int c = cell_index(boids_[static_cast<std::size_t>(i)].pos);
        const int prev = std::atomic_ref<int>(cell_counts_[c]).fetch_add(1, std::memory_order::relaxed);
        if (prev < cell_cap_)
        {
          cell_ids_[c*cell_cap_ + prev] = i;
        }
      }
    });
  }
  for (auto& th : workers) {
    th.join();
  }
}

void BoidSim::compute_forces_range(int begin, int end, std::vector<Vec2>& forces) const {
  std::vector<int> neigh_cells;
  neigh_cells.reserve(9);

  for (int i = begin; i < end; ++i) {
    const Boid& self = boids_[static_cast<std::size_t>(i)];
    neighbors(cell_index(self.pos), neigh_cells);

    Vec2 sep{};
    Vec2 ali{};
    Vec2 coh{};
    int n_sep = 0;
    int n_ali = 0;
    int n_coh = 0;

    for (int cell : neigh_cells) {
      const int count = std::min(cell_counts_[static_cast<std::size_t>(cell)], cell_cap_);
      const int base = cell * cell_cap_;
      for (int k = 0; k < count; ++k) {
        const int j = cell_ids_[static_cast<std::size_t>(base + k)];
        if (j == i) {
          continue;
        }
        const Boid& other = boids_[static_cast<std::size_t>(j)];
        const float dx = toroidal_delta(other.pos.x, self.pos.x, params_.box);
        const float dy = toroidal_delta(other.pos.y, self.pos.y, params_.box);
        const float d2 = dx * dx + dy * dy;

        if (d2 < params_.r_sep * params_.r_sep && d2 > 1e-8f) {
          const float d = std::sqrt(d2);
          sep.x -= dx / d;
          sep.y -= dy / d;
          ++n_sep;
        }
        if (d2 < params_.r_ali * params_.r_ali) {
          ali.x += other.vel.x;
          ali.y += other.vel.y;
          ++n_ali;
        }
        if (d2 < params_.r_coh * params_.r_coh) {
          coh.x += dx;
          coh.y += dy;
          ++n_coh;
        }
      }
    }

    Vec2 force{};
    if (n_sep > 0) {
      sep = normalize_or_zero(sep);
      force.x += sep.x * params_.w_sep;
      force.y += sep.y * params_.w_sep;
    }
    if (n_ali > 0) {
      ali.x /= static_cast<float>(n_ali);
      ali.y /= static_cast<float>(n_ali);
      Vec2 steer{ali.x - self.vel.x, ali.y - self.vel.y};
      steer = clamp_length(normalize_or_zero(steer), params_.max_force);
      force.x += steer.x * params_.w_ali;
      force.y += steer.y * params_.w_ali;
    }
    if (n_coh > 0) {
      coh.x /= static_cast<float>(n_coh);
      coh.y /= static_cast<float>(n_coh);
      Vec2 steer = clamp_length(normalize_or_zero(coh), params_.max_force);
      force.x += steer.x * params_.w_coh;
      force.y += steer.y * params_.w_coh;
    }
    forces[static_cast<std::size_t>(i)] = clamp_length(force, params_.max_force);
  }
}

void BoidSim::integrate(const std::vector<Vec2>& forces, float dt) {
  for (std::size_t i = 0; i < boids_.size(); ++i) {
    Boid& b = boids_[i];
    b.vel.x += forces[i].x * dt;
    b.vel.y += forces[i].y * dt;
    b.vel = clamp_length(b.vel, params_.max_speed);
    smooth_heading(b.heading, b.vel, dt);
    b.pos.x += b.vel.x * dt;
    b.pos.y += b.vel.y * dt;
    b.pos = wrap(b.pos, params_.box);
  }
}

void BoidSim::compute_forces_parallel(std::vector<Vec2>& forces) const {
  forces.assign(boids_.size(), {});
  const int n = static_cast<int>(boids_.size());
  const int w = std::max(1, worker_count_);
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(w));
  for (int t = 0; t < w; ++t) {
    const int begin = n * t / w;
    const int end = n * (t + 1) / w;
    workers.emplace_back([this, begin, end, &forces] {
      compute_forces_range(begin, end, forces);
    });
  }
  for (auto& th : workers) {
    th.join();
  }
}

void BoidSim::step(float dt) {
  const auto t0 = Clock::now();
  rebuild_grid();
  last_grid_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  const auto t1 = Clock::now();
  std::vector<Vec2> forces;
  compute_forces_parallel(forces);
  integrate(forces, dt);
  last_forces_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();
}

void BoidSim::step_sequential_reference(float dt) {
  rebuild_grid_sequential();
  std::vector<Vec2> forces(boids_.size());
  compute_forces_range(0, static_cast<int>(boids_.size()), forces);
  integrate(forces, dt);
}

void BoidSim::pack_positions(std::vector<float>& out) const {
  out.resize(boids_.size() * 4);
  for (std::size_t i = 0; i < boids_.size(); ++i) {
    out[4 * i] = boids_[i].pos.x;
    out[4 * i + 1] = boids_[i].pos.y;
    out[4 * i + 2] = boids_[i].heading.x;
    out[4 * i + 3] = boids_[i].heading.y;
  }
}

bool BoidSim::grid_assignment_ok() const {
  std::vector<int> seen(boids_.size(), 0);
  int total = 0;
  for (std::size_t c = 0; c < cell_counts_.size(); ++c) {
    const int count = std::min(cell_counts_[c], cell_cap_);
    const int base = static_cast<int>(c) * cell_cap_;
    for (int k = 0; k < count; ++k) {
      const int id = cell_ids_[static_cast<std::size_t>(base + k)];
      if (id < 0 || id >= static_cast<int>(boids_.size())) {
        return false;
      }
      ++seen[static_cast<std::size_t>(id)];
      ++total;
    }
  }
  if (total != static_cast<int>(boids_.size())) {
    return false;
  }
  for (int c : seen) {
    if (c != 1) {
      return false;
    }
  }
  return true;
}
