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

#ifndef ISM330DHCX_DRIVER__ISM330DHCX_HPP_
#define ISM330DHCX_DRIVER__ISM330DHCX_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "ism330dhcx_driver/register_bus.hpp"

namespace ism330dhcx_driver
{
// Output data rate codes, shared by CTRL1_XL/CTRL2_G ODR fields and FIFO_CTRL3 batch rates.
enum class Odr : uint8_t
{
  Hz12_5 = 1,
  Hz26 = 2,
  Hz52 = 3,
  Hz104 = 4,
  Hz208 = 5,
  Hz416 = 6,
  Hz833 = 7,
  Hz1666 = 8,
  Hz3332 = 9,
  Hz6667 = 10,
};

enum class AccelRange : uint8_t
{
  G2,
  G4,
  G8,
  G16,
};

enum class GyroRange : uint8_t
{
  Dps125,
  Dps250,
  Dps500,
  Dps1000,
  Dps2000,
  Dps4000,
};

// Parse helpers for configuration values; return std::nullopt for unsupported values.
std::optional<Odr> odr_from_hz(double hz);
std::optional<AccelRange> accel_range_from_g(int64_t g);
std::optional<GyroRange> gyro_range_from_dps(int64_t dps);
double nominal_odr_hz(Odr odr);

struct Config
{
  Odr odr{Odr::Hz208};
  AccelRange accel_range{AccelRange::G4};
  GyroRange gyro_range{GyroRange::Dps500};
};

// One accelerometer + gyroscope sample, in sensor axes and SI units.
struct Sample
{
  std::array<double, 3> linear_acceleration{};  // m/s^2
  std::array<double, 3> angular_velocity{};     // rad/s
};

struct FifoStatus
{
  uint16_t level{0};  // unread FIFO words
  bool watermark{false};
  bool overrun{false};
  bool full{false};
};

struct FifoStatistics
{
  uint64_t words_read{0};
  uint64_t samples{0};
  uint64_t overruns{0};
  uint64_t unpaired_words{0};  // accel or gyro word dropped because its partner was missing
  uint64_t unknown_tags{0};
};

class Ism330dhcx
{
public:
  explicit Ism330dhcx(std::unique_ptr<RegisterBus> bus);

  uint8_t who_am_i();
  bool probe();

  // Software reset; restores all control registers to their defaults. Throws on timeout.
  void reset();
  void configure(const Config & config);

  FifoStatus fifo_status();

  // Drains up to `max_words` FIFO words and appends the complete accel+gyro pairs to `out`.
  // A pair split across two calls is completed on the next call.
  FifoStatus read_fifo(std::vector<Sample> & out, std::size_t max_words = 512);

  double read_temperature();  // degrees Celsius

  // Actual sample rate, corrected with the factory INTERNAL_FREQ_FINE trim. Valid after configure.
  double actual_odr_hz() const { return actual_odr_hz_; }
  const FifoStatistics & statistics() const { return stats_; }
  const Config & config() const { return config_; }

private:
  void set_scales(const Config & config);

  std::unique_ptr<RegisterBus> bus_;
  Config config_{};
  double accel_scale_{0.0};  // m/s^2 per LSB
  double gyro_scale_{0.0};   // rad/s per LSB
  double actual_odr_hz_{0.0};

  std::optional<std::array<int16_t, 3>> pending_accel_;
  std::optional<std::array<int16_t, 3>> pending_gyro_;
  std::vector<uint8_t> fifo_buffer_;
  FifoStatistics stats_{};
};
}  // namespace ism330dhcx_driver

#endif  // ISM330DHCX_DRIVER__ISM330DHCX_HPP_
