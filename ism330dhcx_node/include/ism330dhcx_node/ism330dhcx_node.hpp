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

#ifndef ISM330DHCX_NODE__ISM330DHCX_NODE_HPP_
#define ISM330DHCX_NODE__ISM330DHCX_NODE_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "ism330dhcx_driver/ism330dhcx.hpp"
#include "ism330dhcx_driver/register_bus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace ism330dhcx_node
{
class Ism330dhcxNode : public rclcpp::Node
{
public:
  using BusFactory = std::function<std::unique_ptr<ism330dhcx_driver::RegisterBus>(
    const std::string & device, uint32_t speed_hz)>;

  explicit Ism330dhcxNode(
    BusFactory bus_factory, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  constexpr static const char * const NODE_NAME = "ism330dhcx_node";

  enum class State
  {
    Disconnected,
    Calibrating,
    Running,
  };

  void declare_and_load_parameters();
  void try_initialise();
  void disconnect(const std::string & reason);

  void on_poll();
  void on_health_check();

  void start_calibration();
  void accumulate_calibration(const ism330dhcx_driver::Sample & sample);
  void publish(const ism330dhcx_driver::Sample & sample, const rclcpp::Time & stamp);

  void diagnose_device(diagnostic_updater::DiagnosticStatusWrapper & stat);
  static const char * state_name(State state);

  BusFactory bus_factory_;
  std::unique_ptr<ism330dhcx_driver::Ism330dhcx> imu_;
  State state_{State::Disconnected};
  std::string last_error_{"not initialised"};
  uint64_t connect_count_{0};

  // Parameters
  std::string spi_device_;
  uint32_t spi_speed_hz_{0};
  std::string frame_id_;
  ism330dhcx_driver::Config sensor_config_{};
  int64_t poll_period_ms_{10};
  int64_t reconnect_period_ms_{1000};
  int64_t max_words_per_poll_{512};
  double settle_time_s_{0.1};
  bool calibration_enabled_{true};
  double calibration_duration_s_{2.0};
  double calibration_max_gyro_stddev_{0.01};
  int64_t calibration_max_attempts_{5};
  std::array<double, 3> gyro_bias_{};
  double angular_velocity_variance_{0.0};
  double linear_acceleration_variance_{0.0};

  // Calibration accumulators
  uint64_t calibration_target_samples_{0};
  uint64_t calibration_samples_{0};
  int64_t calibration_attempts_{0};
  std::array<double, 3> gyro_sum_{};
  std::array<double, 3> gyro_sum_sq_{};
  bool bias_calibrated_{false};

  std::vector<ism330dhcx_driver::Sample> samples_;
  uint64_t settle_samples_left_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_connect_attempt_{0, 0, RCL_ROS_TIME};
  double temperature_c_{0.0};
  uint64_t diag_last_overruns_{0};

  double expected_rate_hz_{0.0};
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> imu_rate_diagnostic_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};
}  // namespace ism330dhcx_node

#endif  // ISM330DHCX_NODE__ISM330DHCX_NODE_HPP_
