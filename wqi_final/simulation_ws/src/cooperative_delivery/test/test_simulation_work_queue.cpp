#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"
#include "cooperative_delivery/simulation_work_queue.hpp"

using cooperative_delivery::SimulationWorkQueue;
using namespace std::chrono_literals;

template<typename Future>
void DrainUntilReady(SimulationWorkQueue & queue, Future & future)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    queue.Drain();
    std::this_thread::sleep_for(1ms);
  }
  queue.Close();
}

TEST(SimulationWorkQueue, PhysicsOperationRunsOnlyOnDrainingThread)
{
  SimulationWorkQueue queue;
  std::thread::id executed;
  auto worker = std::async(
    std::launch::async, [&]() {
      return queue.Invoke([&]() {executed = std::this_thread::get_id();});
    });
  DrainUntilReady(queue, worker);
  ASSERT_TRUE(worker.get());
  EXPECT_EQ(executed, std::this_thread::get_id());
}

TEST(SimulationWorkQueue, ExpiredServiceCannotMutatePhysicsLater)
{
  SimulationWorkQueue queue;
  bool changed = false;
  EXPECT_FALSE(queue.Invoke([&]() {changed = true;}, 1ms));
  queue.Drain();
  EXPECT_FALSE(changed);
}

TEST(SimulationWorkQueue, PostedInitialCaptureWaitsForAnActualUpdate)
{
  SimulationWorkQueue queue;
  bool captured = false;
  EXPECT_TRUE(queue.Post([&]() {captured = true;}));
  EXPECT_FALSE(captured);
  queue.Drain();
  EXPECT_TRUE(captured);
}

TEST(SimulationWorkQueue, ShutdownUnblocksCallbacksAndRejectsNewWork)
{
  SimulationWorkQueue queue;
  auto worker = std::async(std::launch::async, [&]() {return queue.Invoke([]() {});});
  queue.Close();
  EXPECT_FALSE(worker.get());
  EXPECT_FALSE(queue.Post([]() {}));
  EXPECT_FALSE(queue.Invoke([]() {}));
}

TEST(SimulationWorkQueue, PhysicsExceptionsReturnToTheWaitingService)
{
  SimulationWorkQueue queue;
  auto worker = std::async(
    std::launch::async, [&]() {
      return queue.Invoke([]() {throw std::logic_error("physics operation failed");});
    });
  DrainUntilReady(queue, worker);
  EXPECT_THROW(worker.get(), std::logic_error);
}
