#include "boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

namespace {

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
  cells_.assign(static_cast<std::size_t>(params_.grid_n * params_.grid_n), {});
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

void BoidSim::rebuild_grid() {
  for (auto& cell : cells_) {
    cell.clear();
  }
  for (int i = 0; i < static_cast<int>(boids_.size()); ++i) {
    cells_[static_cast<std::size_t>(cell_index(boids_[static_cast<std::size_t>(i)].pos))].push_back(i);
  }
}

void BoidSim::compute_forces(std::vector<Vec2>& forces) const {
  forces.assign(boids_.size(), {});
  std::vector<int> neigh_cells;
  neigh_cells.reserve(9);

  for (int i = 0; i < static_cast<int>(boids_.size()); ++i) {
    const Boid& self = boids_[static_cast<std::size_t>(i)];
    neighbors(cell_index(self.pos), neigh_cells);

    Vec2 sep{};
    Vec2 ali{};
    Vec2 coh{};
    int n_sep = 0;
    int n_ali = 0;
    int n_coh = 0;

    for (int cell : neigh_cells) {
      for (int j : cells_[static_cast<std::size_t>(cell)]) {
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

void BoidSim::step(float dt) {
  const auto t0 = std::chrono::steady_clock::now();
  rebuild_grid();
  last_grid_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  const auto t1 = std::chrono::steady_clock::now();
  std::vector<Vec2> forces;
  compute_forces(forces);
  integrate(forces, dt);
  last_forces_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
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
  for (const auto& cell : cells_) {
    for (int id : cell) {
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
