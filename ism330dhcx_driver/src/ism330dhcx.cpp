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

#include "ism330dhcx_driver/ism330dhcx.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

#include "ism330dhcx_driver/registers.hpp"

namespace ism330dhcx_driver
{
namespace
{
constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr int RESET_POLL_ATTEMPTS = 20;
constexpr auto RESET_POLL_PERIOD = std::chrono::milliseconds(1);

constexpr std::array<std::pair<Odr, double>, 10> ODR_TABLE = {{
  {Odr::Hz12_5, 12.5},
  {Odr::Hz26, 26.0},
  {Odr::Hz52, 52.0},
  {Odr::Hz104, 104.0},
  {Odr::Hz208, 208.0},
  {Odr::Hz416, 416.0},
  {Odr::Hz833, 833.0},
  {Odr::Hz1666, 1666.0},
  {Odr::Hz3332, 3332.0},
  {Odr::Hz6667, 6667.0},
}};

int16_t to_int16(const uint8_t * le)
{
  return static_cast<int16_t>(static_cast<uint16_t>(le[0]) | (static_cast<uint16_t>(le[1]) << 8));
}

std::array<int16_t, 3> to_vector3(const uint8_t * le)
{
  return {to_int16(le), to_int16(le + 2), to_int16(le + 4)};
}

uint8_t odr_code(Odr odr) { return static_cast<uint8_t>(odr); }

// CTRL1_XL FS_XL field (with CTRL8_XL.XL_FS_MODE = 0) and sensitivity in mg/LSB.
std::pair<uint8_t, double> accel_fs(AccelRange range)
{
  switch (range) {
    case AccelRange::G2:
      return {0b00, 0.061};
    case AccelRange::G4:
      return {0b10, 0.122};
    case AccelRange::G8:
      return {0b11, 0.244};
    case AccelRange::G16:
      return {0b01, 0.488};
  }
  throw std::invalid_argument("invalid accelerometer range");
}

// CTRL2_G FS bits (FS_G[3:2], FS_125, FS_4000) and sensitivity in mdps/LSB.
std::pair<uint8_t, double> gyro_fs(GyroRange range)
{
  switch (range) {
    case GyroRange::Dps125:
      return {reg::CTRL2_G_FS_125, 4.375};
    case GyroRange::Dps250:
      return {0b00 << 2, 8.75};
    case GyroRange::Dps500:
      return {0b01 << 2, 17.5};
    case GyroRange::Dps1000:
      return {0b10 << 2, 35.0};
    case GyroRange::Dps2000:
      return {0b11 << 2, 70.0};
    case GyroRange::Dps4000:
      return {reg::CTRL2_G_FS_4000, 140.0};
  }
  throw std::invalid_argument("invalid gyroscope range");
}
}  // namespace

std::optional<Odr> odr_from_hz(double hz)
{
  for (const auto & [odr, nominal] : ODR_TABLE) {
    if (std::abs(hz - nominal) < 0.5) {
      return odr;
    }
  }
  return std::nullopt;
}

double nominal_odr_hz(Odr odr)
{
  for (const auto & [code, nominal] : ODR_TABLE) {
    if (code == odr) {
      return nominal;
    }
  }
  throw std::invalid_argument("invalid odr");
}

std::optional<AccelRange> accel_range_from_g(int64_t g)
{
  switch (g) {
    case 2:
      return AccelRange::G2;
    case 4:
      return AccelRange::G4;
    case 8:
      return AccelRange::G8;
    case 16:
      return AccelRange::G16;
    default:
      return std::nullopt;
  }
}

std::optional<GyroRange> gyro_range_from_dps(int64_t dps)
{
  switch (dps) {
    case 125:
      return GyroRange::Dps125;
    case 250:
      return GyroRange::Dps250;
    case 500:
      return GyroRange::Dps500;
    case 1000:
      return GyroRange::Dps1000;
    case 2000:
      return GyroRange::Dps2000;
    case 4000:
      return GyroRange::Dps4000;
    default:
      return std::nullopt;
  }
}

Ism330dhcx::Ism330dhcx(std::unique_ptr<RegisterBus> bus) : bus_(std::move(bus))
{
  if (!bus_) {
    throw std::invalid_argument("Ism330dhcx requires a bus");
  }
  set_scales(config_);
}

uint8_t Ism330dhcx::who_am_i()
{
  uint8_t value = 0;
  bus_->read(reg::WHO_AM_I, &value, 1);
  return value;
}

bool Ism330dhcx::probe() { return who_am_i() == reg::WHO_AM_I_VALUE; }

void Ism330dhcx::reset()
{
  bus_->write(reg::CTRL3_C, reg::CTRL3_C_IF_INC | reg::CTRL3_C_SW_RESET);
  for (int attempt = 0; attempt < RESET_POLL_ATTEMPTS; ++attempt) {
    std::this_thread::sleep_for(RESET_POLL_PERIOD);
    uint8_t ctrl3 = 0;
    bus_->read(reg::CTRL3_C, &ctrl3, 1);
    if ((ctrl3 & reg::CTRL3_C_SW_RESET) == 0) {
      pending_accel_.reset();
      pending_gyro_.reset();
      return;
    }
  }
  throw std::runtime_error("ISM330DHCX software reset timed out");
}

void Ism330dhcx::configure(const Config & config)
{
  const uint8_t accel_bits = accel_fs(config.accel_range).first;
  const uint8_t gyro_bits = gyro_fs(config.gyro_range).first;
  const uint8_t odr = odr_code(config.odr);

  // Bypass mode flushes whatever was batched with the previous configuration.
  bus_->write(reg::FIFO_CTRL4, reg::FIFO_MODE_BYPASS);
  bus_->write(reg::CTRL3_C, reg::CTRL3_C_BDU | reg::CTRL3_C_IF_INC);
  bus_->write(reg::CTRL4_C, reg::CTRL4_C_I2C_DISABLE);

  uint8_t ctrl9 = 0;
  bus_->read(reg::CTRL9_XL, &ctrl9, 1);
  bus_->write(reg::CTRL9_XL, static_cast<uint8_t>(ctrl9 | reg::CTRL9_XL_DEVICE_CONF));

  bus_->write(reg::CTRL1_XL, static_cast<uint8_t>((odr << 4) | (accel_bits << 2)));
  bus_->write(reg::CTRL2_G, static_cast<uint8_t>((odr << 4) | gyro_bits));

  // Batch both sensors at their ODR, no timestamp/temperature words.
  bus_->write(reg::FIFO_CTRL3, static_cast<uint8_t>((odr << 4) | odr));
  bus_->write(reg::FIFO_CTRL4, reg::FIFO_MODE_CONTINUOUS);

  uint8_t freq_fine = 0;
  bus_->read(reg::INTERNAL_FREQ_FINE, &freq_fine, 1);
  // Datasheet: ODR_actual = (6667 + 0.0015 * INTERNAL_FREQ_FINE * 6667) / ODR_coeff,
  // with ODR_coeff = 6667 / ODR_nominal = 2^(10 - odr_code).
  const double fine = static_cast<double>(static_cast<int8_t>(freq_fine));
  actual_odr_hz_ = 6667.0 * (1.0 + 0.0015 * fine) / static_cast<double>(1U << (10U - odr));

  config_ = config;
  set_scales(config);
  pending_accel_.reset();
  pending_gyro_.reset();
}

void Ism330dhcx::set_scales(const Config & config)
{
  accel_scale_ = accel_fs(config.accel_range).second * 1e-3 * STANDARD_GRAVITY;
  gyro_scale_ = gyro_fs(config.gyro_range).second * 1e-3 * DEG_TO_RAD;
}

FifoStatus Ism330dhcx::fifo_status()
{
  uint8_t status[2] = {0, 0};
  bus_->read(reg::FIFO_STATUS1, status, sizeof(status));
  FifoStatus result;
  result.level = static_cast<uint16_t>(
    status[0] | ((status[1] & reg::FIFO_STATUS2_DIFF_FIFO_HI_MASK) << 8));
  result.watermark = (status[1] & reg::FIFO_STATUS2_WTM_IA) != 0;
  result.overrun = (status[1] & reg::FIFO_STATUS2_OVR_IA) != 0;
  result.full = (status[1] & reg::FIFO_STATUS2_FULL_IA) != 0;
  return result;
}

FifoStatus Ism330dhcx::read_fifo(std::vector<Sample> & out, std::size_t max_words)
{
  const FifoStatus status = fifo_status();
  if (status.overrun) {
    ++stats_.overruns;
  }
  const std::size_t words = std::min<std::size_t>(status.level, max_words);
  if (words == 0) {
    return status;
  }

  fifo_buffer_.resize(words * reg::FIFO_WORD_SIZE);
  bus_->read_blocks(reg::FIFO_DATA_OUT_TAG, reg::FIFO_WORD_SIZE, words, fifo_buffer_.data());
  stats_.words_read += words;

  for (std::size_t i = 0; i < words; ++i) {
    const uint8_t * word = fifo_buffer_.data() + i * reg::FIFO_WORD_SIZE;
    const uint8_t tag = static_cast<uint8_t>(word[0] >> reg::FIFO_TAG_SHIFT);
    if (tag == reg::FIFO_TAG_GYRO) {
      if (pending_gyro_) {
        ++stats_.unpaired_words;
      }
      pending_gyro_ = to_vector3(word + 1);
    } else if (tag == reg::FIFO_TAG_ACCEL) {
      if (pending_accel_) {
        ++stats_.unpaired_words;
      }
      pending_accel_ = to_vector3(word + 1);
    } else {
      ++stats_.unknown_tags;
      continue;
    }

    if (pending_accel_ && pending_gyro_) {
      Sample sample;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        sample.linear_acceleration[axis] = (*pending_accel_)[axis] * accel_scale_;
        sample.angular_velocity[axis] = (*pending_gyro_)[axis] * gyro_scale_;
      }
      out.push_back(sample);
      ++stats_.samples;
      pending_accel_.reset();
      pending_gyro_.reset();
    }
  }
  return status;
}

double Ism330dhcx::read_temperature()
{
  uint8_t raw[2] = {0, 0};
  bus_->read(reg::OUT_TEMP_L, raw, sizeof(raw));
  return 25.0 + static_cast<double>(to_int16(raw)) / 256.0;
}
}  // namespace ism330dhcx_driver
