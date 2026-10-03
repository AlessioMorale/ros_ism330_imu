// Copyright 2026 Alessio Morale <alessiomorale-at-gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// SPDX-FileCopyrightText: 2026 Alessio Morale <alessiomorale-at-gmail.com>
// SPDX-License-Identifier: mit
//

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ism330dhcx_driver/registers.hpp"
#include "ism330dhcx_driver/testing/fake_register_bus.hpp"
#include "ism330dhcx_node/ism330dhcx_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;  // NOLINT
using ism330dhcx_driver::testing::FakeRegisterBus;
namespace reg = ism330dhcx_driver::reg;

namespace
{
// Hands the node a fresh bus object on every (re)connect, all backed by the same fake device.
class SharedBus : public ism330dhcx_driver::RegisterBus
{
public:
  explicit SharedBus(std::shared_ptr<FakeRegisterBus> fake) : fake_(std::move(fake)) {}
  void read(uint8_t address, uint8_t * data, std::size_t len) override
  {
    fake_->read(address, data, len);
  }
  void write(uint8_t address, uint8_t value) override { fake_->write(address, value); }

private:
  std::shared_ptr<FakeRegisterBus> fake_;
};

class Ism330dhcxNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(const std::vector<rclcpp::Parameter> & parameters)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    options.append_parameter_override("poll_period_ms", 2);
    if (!has_parameter(parameters, "settle_time_s")) {
      options.append_parameter_override("settle_time_s", 0.0);
    }
    node_ = std::make_shared<ism330dhcx_node::Ism330dhcxNode>(
      [this](const std::string & device, uint32_t) {
        ++factory_calls_;
        EXPECT_EQ(device, "/dev/spidev0.0");
        return std::make_unique<SharedBus>(fake_);
      },
      options);

    listener_ = std::make_shared<rclcpp::Node>("listener");
    subscription_ = listener_->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 100, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { received_.push_back(*msg); });
    executor_.add_node(node_);
    executor_.add_node(listener_);
  }

  static bool has_parameter(const std::vector<rclcpp::Parameter> & parameters, const char * name)
  {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == name) {
        return true;
      }
    }
    return false;
  }

  void TearDown() override
  {
    if (node_) {
      executor_.remove_node(node_);
      executor_.remove_node(listener_);
    }
  }

  void push_samples(int count, int16_t gyro_z, int16_t accel_z)
  {
    for (int i = 0; i < count; ++i) {
      fake_->push_gyro(0, 0, gyro_z);
      fake_->push_accel(0, 0, accel_z);
    }
  }

  bool spin_until(const std::function<bool()> & done, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
      if (done()) {
        return true;
      }
    }
    return done();
  }

  std::shared_ptr<FakeRegisterBus> fake_ = std::make_shared<FakeRegisterBus>();
  int factory_calls_{0};
  std::shared_ptr<ism330dhcx_node::Ism330dhcxNode> node_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::vector<sensor_msgs::msg::Imu> received_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};
}  // namespace

TEST_F(Ism330dhcxNodeTest, CalibratesBiasThenPublishes)
{
  start({rclcpp::Parameter("calibration.duration_s", 0.05)});
  // 0.05 s at ~208 Hz -> 11 calibration samples; the rest are published bias-corrected.
  push_samples(30, 100, 8197);

  ASSERT_TRUE(spin_until([this]() { return received_.size() >= 19; }));
  EXPECT_EQ(received_.size(), 19U);
  for (std::size_t i = 0; i < received_.size(); ++i) {
    const auto & msg = received_[i];
    EXPECT_EQ(msg.header.frame_id, "imu_link");
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0);
    EXPECT_NEAR(msg.angular_velocity.z, 0.0, 1e-9);
    EXPECT_NEAR(msg.linear_acceleration.z, 9.80665, 1e-2);
    EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
    if (i > 0) {
      EXPECT_GT(rclcpp::Time(msg.header.stamp), rclcpp::Time(received_[i - 1].header.stamp));
    }
  }
}

TEST_F(Ism330dhcxNodeTest, WaitsForDeviceThenConnects)
{
  fake_->set_register(reg::WHO_AM_I, 0x00);
  start(
    {rclcpp::Parameter("calibration.enabled", false),
     rclcpp::Parameter("reconnect_period_ms", 100)});
  push_samples(5, 0, 0);

  EXPECT_FALSE(spin_until([this]() { return !received_.empty(); }, 300ms));
  EXPECT_GE(factory_calls_, 2);

  fake_->set_register(reg::WHO_AM_I, reg::WHO_AM_I_VALUE);
  EXPECT_TRUE(spin_until([this]() { return received_.size() == 5; }));
}

TEST_F(Ism330dhcxNodeTest, ReconnectsAfterBusFailure)
{
  start(
    {rclcpp::Parameter("calibration.enabled", false),
     rclcpp::Parameter("gyro_bias", std::vector<double>{0.0, 0.0, 0.5})});
  push_samples(3, 0, 0);
  ASSERT_TRUE(spin_until([this]() { return received_.size() == 3; }));
  EXPECT_NEAR(received_.back().angular_velocity.z, -0.5, 1e-9);
  EXPECT_EQ(factory_calls_, 1);

  fake_->set_failing(true);
  spin_until([]() { return false; }, 100ms);
  fake_->set_failing(false);
  push_samples(3, 0, 0);
  EXPECT_TRUE(spin_until([this]() { return received_.size() == 6; }, 3s));
  EXPECT_EQ(factory_calls_, 2);
}

TEST_F(Ism330dhcxNodeTest, DiscardsSamplesWhileSettling)
{
  // 0.05 s at ~208 Hz -> the first 11 samples are dropped.
  start(
    {rclcpp::Parameter("calibration.enabled", false),
     rclcpp::Parameter("settle_time_s", 0.05)});
  push_samples(20, 0, 0);
  EXPECT_TRUE(spin_until([this]() { return received_.size() == 9; }));
  EXPECT_FALSE(spin_until([this]() { return received_.size() > 9; }, 100ms));
}
