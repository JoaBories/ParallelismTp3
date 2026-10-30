#include "thread_pool.hpp"

#include <utility>

ThreadPool::ThreadPool(std::size_t worker_count) : work_(0), done_(0) {
  if (worker_count == 0) {
    throw std::invalid_argument("worker_count must be > 0");
  }
  workers_.reserve(worker_count);
  for (std::size_t i = 0; i < worker_count; ++i) {
    workers_.emplace_back([this] { worker_loop(); });
  }
}

ThreadPool::~ThreadPool() {
  close();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void ThreadPool::close() {
  closed_.store(true, std::memory_order_release);
  for (std::size_t i = 0; i < workers_.size(); ++i) {
    work_.release();
  }
}

void ThreadPool::submit(std::function<void()> job) {
  if (closed_.load(std::memory_order_acquire)) {
    throw std::runtime_error("pool closed");
  }
  const std::size_t i = write_.load(std::memory_order_relaxed);
  if (i - read_.load(std::memory_order_acquire) >= kCap) {
    throw std::runtime_error("pool queue full");
  }
  pending_.fetch_add(1, std::memory_order_relaxed);
  slots_[i % kCap] = std::move(job);
  write_.store(i + 1, std::memory_order_release);
  work_.release();
}

void ThreadPool::wait() {
  // Dort vraiment (comme join) : ne vole pas de cœur aux workers.
  while (pending_.load(std::memory_order_acquire) > 0) {
    done_.acquire();
  }
}

void ThreadPool::worker_loop() {
  for (;;) {
    work_.acquire();
    if (closed_.load(std::memory_order_acquire)) {
      return;
    }
    const std::size_t i = read_.fetch_add(1, std::memory_order_acq_rel);
    std::function<void()> job = std::move(slots_[i % kCap]);
    slots_[i % kCap] = nullptr;
    job();
    if (pending_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      done_.release();
    }
  }
}
