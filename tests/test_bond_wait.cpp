#include <gtest/gtest.h>
#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

TEST(BondWait, FormationAndBreakWhileCallbacksTransitionState)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("bond_wait_regression");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&executor]() {executor.spin();});
  for (int i = 0; i < 20; ++i) {
    auto first = std::make_shared<bond::Bond>("bond_wait_test", std::to_string(i), node);
    auto second = std::make_shared<bond::Bond>("bond_wait_test", std::to_string(i), node);
    first->setHeartbeatPeriod(0.01);
    second->setHeartbeatPeriod(0.01);
    first->start();
    second->start();
    EXPECT_TRUE(first->waitUntilFormed(rclcpp::Duration(5s)));
    EXPECT_TRUE(second->waitUntilFormed(rclcpp::Duration(5s)));
    first->breakBond();
    EXPECT_TRUE(second->waitUntilBroken(rclcpp::Duration(5s)));
    EXPECT_TRUE(first->waitUntilBroken(rclcpp::Duration(5s)));
  }
  executor.cancel();
  spin.join();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
}
