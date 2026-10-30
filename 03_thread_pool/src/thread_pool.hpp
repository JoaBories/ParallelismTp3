#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <semaphore>

// File circulaire : write_/read_ atomiques.
// work_  : réveille un worker quand une tâche est poussée.
// done_  : réveille wait() quand pending_ repasse à 0 (le main dort, comme join).
class ThreadPool {
 public:
  explicit ThreadPool(std::size_t worker_count);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void submit(std::function<void()> job);
  void wait();
  void close();

 private:
  void worker_loop();

  static constexpr std::size_t kCap = 256;

  std::function<void()> slots_[kCap];
  std::atomic<std::size_t> write_{0};
  std::atomic<std::size_t> read_{0};
  std::atomic<int> pending_{0};
  std::counting_semaphore<> work_;
  std::counting_semaphore<> done_;
  std::atomic<bool> closed_{false};
  std::vector<std::thread> workers_;
};
