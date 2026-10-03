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

#include "ism330dhcx_node/ism330dhcx_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "ism330dhcx_driver/registers.hpp"

namespace ism330dhcx_node
{
namespace
{
std::string format_hex(uint8_t value)
{
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "0x%02X", value);
  return buffer;
}
}  // namespace

Ism330dhcxNode::Ism330dhcxNode(BusFactory bus_factory, const rclcpp::NodeOptions & options)
: Node(NODE_NAME, options), bus_factory_(std::move(bus_factory))
{
  if (!bus_factory_) {
    throw std::invalid_argument("Ism330dhcxNode requires a bus factory");
  }
  declare_and_load_parameters();

  const auto imu_topic = this->declare_parameter<std::string>("imu_topic", "imu");
  const auto temperature_topic =
    this->declare_parameter<std::string>("temperature_topic", "imu/temperature");
  // Reliable so that both reliable and best-effort subscribers match; deep enough for a burst.
  imu_publisher_ =
    this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(50)));
  temperature_publisher_ =
    this->create_publisher<sensor_msgs::msg::Temperature>(temperature_topic, 10);

  expected_rate_hz_ = ism330dhcx_driver::nominal_odr_hz(sensor_config_.odr);
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("ISM330DHCX@" + spi_device_);
  diagnostic_updater_->add("device", this, &Ism330dhcxNode::diagnose_device);
  imu_rate_diagnostic_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    imu_topic, *diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&expected_rate_hz_, &expected_rate_hz_, 0.1, 10),
    this->get_clock());

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_period_ms_), [this]() { this->on_poll(); });
  health_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), [this]() { this->on_health_check(); });

  RCLCPP_INFO(
    this->get_logger(), "ISM330DHCX node started on %s (%.1f Hz, frame '%s')",
    spi_device_.c_str(), expected_rate_hz_, frame_id_.c_str());
}

void Ism330dhcxNode::declare_and_load_parameters()
{
  spi_device_ = this->declare_parameter<std::string>("spi.device", "/dev/spidev0.0");
  const auto speed_hz = this->declare_parameter<int64_t>("spi.speed_hz", 8'000'000);
  if (speed_hz <= 0 || speed_hz > 10'000'000) {
    throw std::invalid_argument("spi.speed_hz must be in (0, 10 MHz]");
  }
  spi_speed_hz_ = static_cast<uint32_t>(speed_hz);

  frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");

  const auto odr_hz = this->declare_parameter<double>("odr_hz", 208.0);
  const auto odr = ism330dhcx_driver::odr_from_hz(odr_hz);
  if (!odr) {
    throw std::invalid_argument(
      "odr_hz must be one of 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6667");
  }
  const auto accel_range =
    ism330dhcx_driver::accel_range_from_g(this->declare_parameter<int64_t>("accel_range_g", 4));
  if (!accel_range) {
    throw std::invalid_argument("accel_range_g must be one of 2, 4, 8, 16");
  }
  const auto gyro_range = ism330dhcx_driver::gyro_range_from_dps(
    this->declare_parameter<int64_t>("gyro_range_dps", 500));
  if (!gyro_range) {
    throw std::invalid_argument("gyro_range_dps must be one of 125, 250, 500, 1000, 2000, 4000");
  }
  sensor_config_ = ism330dhcx_driver::Config{*odr, *accel_range, *gyro_range};

  poll_period_ms_ = std::max<int64_t>(1, this->declare_parameter<int64_t>("poll_period_ms", 10));
  reconnect_period_ms_ =
    std::max<int64_t>(100, this->declare_parameter<int64_t>("reconnect_period_ms", 1000));
  max_words_per_poll_ =
    std::max<int64_t>(2, this->declare_parameter<int64_t>("max_words_per_poll", 512));

  settle_time_s_ = std::max(0.0, this->declare_parameter<double>("settle_time_s", 0.1));

  calibration_enabled_ = this->declare_parameter<bool>("calibration.enabled", true);
  calibration_duration_s_ = this->declare_parameter<double>("calibration.duration_s", 2.0);
  calibration_max_gyro_stddev_ =
    this->declare_parameter<double>("calibration.max_gyro_stddev", 0.01);
  calibration_max_attempts_ =
    std::max<int64_t>(1, this->declare_parameter<int64_t>("calibration.max_attempts", 5));

  const auto bias =
    this->declare_parameter<std::vector<double>>("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
  if (bias.size() != 3) {
    throw std::invalid_argument("gyro_bias must have 3 elements");
  }
  std::copy(bias.begin(), bias.end(), gyro_bias_.begin());

  const double gyro_stddev =
    this->declare_parameter<double>("noise.angular_velocity_stddev", 0.001);
  const double accel_stddev =
    this->declare_parameter<double>("noise.linear_acceleration_stddev", 0.01);
  angular_velocity_variance_ = gyro_stddev * gyro_stddev;
  linear_acceleration_variance_ = accel_stddev * accel_stddev;
}

void Ism330dhcxNode::try_initialise()
{
  try {
    auto imu = std::make_unique<ism330dhcx_driver::Ism330dhcx>(
      bus_factory_(spi_device_, spi_speed_hz_));
    // The first transfer after switching the controller to SPI mode 3 can read back 0x00.
    imu->who_am_i();
    const uint8_t id = imu->who_am_i();
    if (id != ism330dhcx_driver::reg::WHO_AM_I_VALUE) {
      disconnect(
        "unexpected WHO_AM_I " + format_hex(id) + " (expected " +
        format_hex(ism330dhcx_driver::reg::WHO_AM_I_VALUE) + "), check wiring and power");
      return;
    }
    imu->reset();
    imu->configure(sensor_config_);
    imu_ = std::move(imu);
  } catch (const std::exception & e) {
    disconnect(std::string("initialisation failed: ") + e.what());
    return;
  }

  ++connect_count_;
  last_error_.clear();
  diag_last_overruns_ = 0;
  expected_rate_hz_ = imu_->actual_odr_hz();
  last_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  settle_samples_left_ = static_cast<uint64_t>(std::ceil(settle_time_s_ * imu_->actual_odr_hz()));
  RCLCPP_INFO(
    this->get_logger(), "ISM330DHCX configured: ODR %.2f Hz (trimmed)", imu_->actual_odr_hz());

  // The bias survives reconnects: the robot may be moving when the link comes back.
  if (calibration_enabled_ && !bias_calibrated_) {
    start_calibration();
  } else {
    state_ = State::Running;
  }
}

void Ism330dhcxNode::disconnect(const std::string & reason)
{
  if (reason != last_error_) {
    RCLCPP_ERROR(this->get_logger(), "ISM330DHCX: %s", reason.c_str());
  }
  imu_.reset();
  state_ = State::Disconnected;
  last_error_ = reason;
  next_connect_attempt_ = this->now() + std::chrono::milliseconds(reconnect_period_ms_);
}

void Ism330dhcxNode::on_poll()
{
  if (state_ == State::Disconnected) {
    if (this->now() >= next_connect_attempt_) {
      try_initialise();
    }
    return;
  }

  samples_.clear();
  ism330dhcx_driver::FifoStatus status;
  try {
    status = imu_->read_fifo(samples_, static_cast<std::size_t>(max_words_per_poll_));
  } catch (const std::exception & e) {
    disconnect(std::string("FIFO read failed: ") + e.what());
    return;
  }
  if (samples_.empty()) {
    return;
  }

  // The newest sample read was produced just before now; older ones (and those still waiting in
  // the FIFO when the read was capped) are spaced by the trimmed sample period.
  const rclcpp::Time read_time = this->now();
  const double period_s = 1.0 / imu_->actual_odr_hz();
  const std::size_t words_left =
    status.level - std::min<std::size_t>(status.level, static_cast<std::size_t>(max_words_per_poll_));
  const std::size_t samples_left = words_left / 2;
  const std::size_t count = samples_.size();
  for (std::size_t i = 0; i < count; ++i) {
    const auto age = static_cast<double>(count - 1 - i + samples_left) * period_s;
    rclcpp::Time stamp = read_time - rclcpp::Duration::from_seconds(age);
    if (stamp <= last_stamp_) {
      stamp = last_stamp_ + rclcpp::Duration::from_nanoseconds(1000);
    }
    last_stamp_ = stamp;

    // Samples produced while the gyro is still starting up are not valid.
    if (settle_samples_left_ > 0) {
      --settle_samples_left_;
      continue;
    }
    if (state_ == State::Calibrating) {
      accumulate_calibration(samples_[i]);
    } else {
      publish(samples_[i], stamp);
    }
  }
}

void Ism330dhcxNode::on_health_check()
{
  if (state_ == State::Disconnected) {
    return;
  }
  try {
    const uint8_t id = imu_->who_am_i();
    if (id != ism330dhcx_driver::reg::WHO_AM_I_VALUE) {
      disconnect("device lost: WHO_AM_I reads " + format_hex(id));
      return;
    }
    temperature_c_ = imu_->read_temperature();
  } catch (const std::exception & e) {
    disconnect(std::string("health check failed: ") + e.what());
    return;
  }

  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.temperature = temperature_c_;
  temperature_publisher_->publish(msg);
}

void Ism330dhcxNode::start_calibration()
{
  calibration_target_samples_ = static_cast<uint64_t>(
    std::max(1.0, std::ceil(calibration_duration_s_ * imu_->actual_odr_hz())));
  calibration_samples_ = 0;
  gyro_sum_ = {};
  gyro_sum_sq_ = {};
  state_ = State::Calibrating;
  RCLCPP_INFO(
    this->get_logger(), "Calibrating gyro bias over %.1f s, keep the robot still",
    calibration_duration_s_);
}

void Ism330dhcxNode::accumulate_calibration(const ism330dhcx_driver::Sample & sample)
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    gyro_sum_[axis] += sample.angular_velocity[axis];
    gyro_sum_sq_[axis] += sample.angular_velocity[axis] * sample.angular_velocity[axis];
  }
  if (++calibration_samples_ < calibration_target_samples_) {
    return;
  }

  const auto n = static_cast<double>(calibration_samples_);
  std::array<double, 3> mean{};
  double max_stddev = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    mean[axis] = gyro_sum_[axis] / n;
    const double variance = gyro_sum_sq_[axis] / n - mean[axis] * mean[axis];
    max_stddev = std::max(max_stddev, std::sqrt(std::max(variance, 0.0)));
  }
  ++calibration_attempts_;

  if (max_stddev <= calibration_max_gyro_stddev_) {
    gyro_bias_ = mean;
    bias_calibrated_ = true;
    state_ = State::Running;
    RCLCPP_INFO(
      this->get_logger(), "Gyro bias [%.5f, %.5f, %.5f] rad/s (stddev %.5f)", mean[0], mean[1],
      mean[2], max_stddev);
  } else if (calibration_attempts_ >= calibration_max_attempts_) {
    state_ = State::Running;
    RCLCPP_WARN(
      this->get_logger(),
      "Gyro calibration gave up after %ld attempts (stddev %.5f > %.5f rad/s); "
      "using the configured gyro_bias",
      static_cast<long>(calibration_attempts_), max_stddev, calibration_max_gyro_stddev_);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Robot moving during gyro calibration (stddev %.5f rad/s), retrying",
      max_stddev);
    start_calibration();
  }
}

void Ism330dhcxNode::publish(const ism330dhcx_driver::Sample & sample, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.orientation_covariance[0] = -1.0;  // no orientation estimate

  msg.angular_velocity.x = sample.angular_velocity[0] - gyro_bias_[0];
  msg.angular_velocity.y = sample.angular_velocity[1] - gyro_bias_[1];
  msg.angular_velocity.z = sample.angular_velocity[2] - gyro_bias_[2];
  msg.linear_acceleration.x = sample.linear_acceleration[0];
  msg.linear_acceleration.y = sample.linear_acceleration[1];
  msg.linear_acceleration.z = sample.linear_acceleration[2];
  for (std::size_t i = 0; i < 3; ++i) {
    msg.angular_velocity_covariance[i * 4] = angular_velocity_variance_;
    msg.linear_acceleration_covariance[i * 4] = linear_acceleration_variance_;
  }

  imu_publisher_->publish(msg);
  imu_rate_diagnostic_->tick();
}

const char * Ism330dhcxNode::state_name(State state)
{
  switch (state) {
    case State::Disconnected:
      return "disconnected";
    case State::Calibrating:
      return "calibrating";
    case State::Running:
      return "running";
  }
  return "unknown";
}

void Ism330dhcxNode::diagnose_device(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  switch (state_) {
    case State::Disconnected:
      stat.summary(DiagnosticStatus::ERROR, last_error_);
      break;
    case State::Calibrating:
      stat.summary(DiagnosticStatus::WARN, "calibrating gyro bias");
      break;
    case State::Running:
      stat.summary(DiagnosticStatus::OK, "running");
      break;
  }

  stat.add("state", state_name(state_));
  stat.add("spi device", spi_device_);
  stat.add("connections", connect_count_);
  stat.add("gyro bias source", bias_calibrated_ ? "calibrated" : "configured");
  stat.addf("gyro bias (rad/s)", "%.5f %.5f %.5f", gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);

  if (!imu_) {
    return;
  }
  const auto & stats = imu_->statistics();
  if (stats.overruns > diag_last_overruns_ && state_ == State::Running) {
    stat.mergeSummary(DiagnosticStatus::WARN, "FIFO overrun, polling too slow");
  }
  diag_last_overruns_ = stats.overruns;

  stat.addf("odr (Hz)", "%.2f", imu_->actual_odr_hz());
  stat.addf("temperature (C)", "%.2f", temperature_c_);
  stat.add("samples", stats.samples);
  stat.add("fifo overruns", stats.overruns);
  stat.add("unpaired words", stats.unpaired_words);
  stat.add("unknown tags", stats.unknown_tags);
}
}  // namespace ism330dhcx_node
