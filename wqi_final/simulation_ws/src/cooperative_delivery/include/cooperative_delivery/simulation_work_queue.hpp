#ifndef COOPERATIVE_DELIVERY__SIMULATION_WORK_QUEUE_HPP_
#define COOPERATIVE_DELIVERY__SIMULATION_WORK_QUEUE_HPP_

#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace cooperative_delivery
{
// ROS callbacks wait; the Gazebo update thread alone drains physics mutations.
class SimulationWorkQueue
{
public:
  bool Invoke(
    std::function<void()> function,
    std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    auto work = std::make_shared<Work>();
    work->function = std::move(function);
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    pending_.push_back(work);
    condition_.wait_for(lock, timeout, [&]() {return work->done || closed_;});
    if (!work->done) {
      work->canceled = true;
      return false;
    }
    if (work->error) {
      std::rethrow_exception(work->error);
    }
    return true;
  }

  bool Post(std::function<void()> function)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    auto work = std::make_shared<Work>();
    work->function = std::move(function);
    pending_.push_back(work);
    return true;
  }

  void Drain()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty()) {
      auto work = pending_.front();
      pending_.pop_front();
      if (work->canceled || closed_) {
        continue;
      }
      try {
        work->function();
      } catch (...) {
        work->error = std::current_exception();
      }
      work->done = true;
      condition_.notify_all();
    }
  }

  void Close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    pending_.clear();
    condition_.notify_all();
  }

private:
  struct Work
  {
    std::function<void()> function;
    std::exception_ptr error;
    bool done{false};
    bool canceled{false};
  };
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::shared_ptr<Work>> pending_;
  bool closed_{false};
};
}  // namespace cooperative_delivery

#endif  // COOPERATIVE_DELIVERY__SIMULATION_WORK_QUEUE_HPP_
