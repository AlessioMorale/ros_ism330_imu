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

#include <cmath>
#include <memory>
#include <vector>

#include "ism330dhcx_driver/ism330dhcx.hpp"
#include "ism330dhcx_driver/registers.hpp"
#include "ism330dhcx_driver/testing/fake_register_bus.hpp"

using ism330dhcx_driver::AccelRange;
using ism330dhcx_driver::Config;
using ism330dhcx_driver::GyroRange;
using ism330dhcx_driver::Ism330dhcx;
using ism330dhcx_driver::Odr;
using ism330dhcx_driver::Sample;
using ism330dhcx_driver::testing::FakeRegisterBus;
namespace reg = ism330dhcx_driver::reg;

namespace
{
class Ism330dhcxTest : public ::testing::Test
{
protected:
  Ism330dhcxTest()
  {
    auto bus = std::make_unique<FakeRegisterBus>();
    bus_ = bus.get();
    imu_ = std::make_unique<Ism330dhcx>(std::move(bus));
  }

  FakeRegisterBus * bus_;
  std::unique_ptr<Ism330dhcx> imu_;
};
}  // namespace

TEST_F(Ism330dhcxTest, ProbeChecksWhoAmI)
{
  EXPECT_TRUE(imu_->probe());
  bus_->set_register(reg::WHO_AM_I, 0x00);
  EXPECT_FALSE(imu_->probe());
}

TEST_F(Ism330dhcxTest, ResetSetsSwResetBit)
{
  imu_->reset();
  const auto writes = bus_->writes();
  ASSERT_FALSE(writes.empty());
  EXPECT_EQ(writes.front().first, reg::CTRL3_C);
  EXPECT_TRUE(writes.front().second & reg::CTRL3_C_SW_RESET);
}

TEST_F(Ism330dhcxTest, ConfigureProgramsRegisters)
{
  imu_->configure(Config{Odr::Hz208, AccelRange::G4, GyroRange::Dps500});

  EXPECT_EQ(bus_->get_register(reg::CTRL3_C), reg::CTRL3_C_BDU | reg::CTRL3_C_IF_INC);
  EXPECT_EQ(bus_->get_register(reg::CTRL1_XL), 0x58);  // ODR 208 Hz, FS +-4 g
  EXPECT_EQ(bus_->get_register(reg::CTRL2_G), 0x54);   // ODR 208 Hz, FS 500 dps
  EXPECT_EQ(bus_->get_register(reg::FIFO_CTRL3), 0x55);
  EXPECT_EQ(bus_->get_register(reg::FIFO_CTRL4), reg::FIFO_MODE_CONTINUOUS);
  EXPECT_EQ(bus_->get_register(reg::CTRL9_XL), 0xE2);
}

TEST_F(Ism330dhcxTest, ConfigureGyroExtremeRanges)
{
  imu_->configure(Config{Odr::Hz104, AccelRange::G16, GyroRange::Dps125});
  EXPECT_EQ(bus_->get_register(reg::CTRL1_XL), 0x44);
  EXPECT_EQ(bus_->get_register(reg::CTRL2_G), 0x42);
  imu_->configure(Config{Odr::Hz104, AccelRange::G8, GyroRange::Dps4000});
  EXPECT_EQ(bus_->get_register(reg::CTRL1_XL), 0x4C);
  EXPECT_EQ(bus_->get_register(reg::CTRL2_G), 0x41);
}

TEST_F(Ism330dhcxTest, ActualOdrUsesFrequencyTrim)
{
  imu_->configure(Config{});
  EXPECT_NEAR(imu_->actual_odr_hz(), 6667.0 / 32.0, 1e-9);

  bus_->set_register(reg::INTERNAL_FREQ_FINE, static_cast<uint8_t>(-10));
  imu_->configure(Config{});
  EXPECT_NEAR(imu_->actual_odr_hz(), 6667.0 * (1.0 - 0.015) / 32.0, 1e-9);
}

TEST_F(Ism330dhcxTest, ReadFifoPairsAndScalesSamples)
{
  imu_->configure(Config{Odr::Hz208, AccelRange::G4, GyroRange::Dps500});
  bus_->push_gyro(1000, -1000, 0);
  bus_->push_accel(0, 0, 8197);  // ~1 g at 0.122 mg/LSB
  bus_->push_accel(100, 0, 0);
  bus_->push_gyro(0, 0, 2000);

  std::vector<Sample> samples;
  const auto status = imu_->read_fifo(samples);
  EXPECT_EQ(status.level, 4);
  ASSERT_EQ(samples.size(), 2U);

  const double gyro_lsb = 17.5e-3 * M_PI / 180.0;
  const double accel_lsb = 0.122e-3 * 9.80665;
  EXPECT_NEAR(samples[0].angular_velocity[0], 1000 * gyro_lsb, 1e-12);
  EXPECT_NEAR(samples[0].angular_velocity[1], -1000 * gyro_lsb, 1e-12);
  EXPECT_NEAR(samples[0].linear_acceleration[2], 9.80665, 1e-2);
  EXPECT_NEAR(samples[1].linear_acceleration[0], 100 * accel_lsb, 1e-12);
  EXPECT_NEAR(samples[1].angular_velocity[2], 2000 * gyro_lsb, 1e-12);
  EXPECT_EQ(imu_->statistics().samples, 2U);
  EXPECT_EQ(bus_->fifo_size(), 0U);
}

TEST_F(Ism330dhcxTest, PairSpanningTwoReadsIsCompleted)
{
  imu_->configure(Config{});
  bus_->push_gyro(1, 2, 3);

  std::vector<Sample> samples;
  imu_->read_fifo(samples);
  EXPECT_TRUE(samples.empty());

  bus_->push_accel(4, 5, 6);
  imu_->read_fifo(samples);
  EXPECT_EQ(samples.size(), 1U);
}

TEST_F(Ism330dhcxTest, UnpairedWordIsDroppedAndCounted)
{
  imu_->configure(Config{});
  bus_->push_gyro(1, 1, 1);
  bus_->push_gyro(2, 2, 2);
  bus_->push_accel(3, 3, 3);
  bus_->push_word(0x04, 0, 0, 0);  // timestamp word, not batched by this driver

  std::vector<Sample> samples;
  imu_->read_fifo(samples);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_GT(samples[0].angular_velocity[0], 0.0);
  EXPECT_NEAR(samples[0].angular_velocity[0] / samples[0].angular_velocity[1], 1.0, 1e-12);
  EXPECT_EQ(imu_->statistics().unpaired_words, 1U);
  EXPECT_EQ(imu_->statistics().unknown_tags, 1U);
}

TEST_F(Ism330dhcxTest, ReadFifoHonoursMaxWords)
{
  imu_->configure(Config{});
  for (int i = 0; i < 5; ++i) {
    bus_->push_gyro(0, 0, 0);
    bus_->push_accel(0, 0, 0);
  }
  std::vector<Sample> samples;
  imu_->read_fifo(samples, 4);
  EXPECT_EQ(samples.size(), 2U);
  EXPECT_EQ(bus_->fifo_size(), 6U);
}

TEST_F(Ism330dhcxTest, OverrunIsCounted)
{
  imu_->configure(Config{});
  bus_->set_overrun(true);
  std::vector<Sample> samples;
  EXPECT_TRUE(imu_->read_fifo(samples).overrun);
  EXPECT_EQ(imu_->statistics().overruns, 1U);
}

TEST_F(Ism330dhcxTest, TemperatureConversion)
{
  bus_->set_register(reg::OUT_TEMP_L, 0x00);
  bus_->set_register(reg::OUT_TEMP_L + 1, 0x02);  // 512 LSB -> +2 degC
  EXPECT_NEAR(imu_->read_temperature(), 27.0, 1e-9);
}

TEST(Ism330dhcxParse, ConfigValues)
{
  EXPECT_EQ(ism330dhcx_driver::odr_from_hz(208.0), Odr::Hz208);
  EXPECT_EQ(ism330dhcx_driver::odr_from_hz(12.5), Odr::Hz12_5);
  EXPECT_FALSE(ism330dhcx_driver::odr_from_hz(200.0).has_value());
  EXPECT_EQ(ism330dhcx_driver::accel_range_from_g(8), AccelRange::G8);
  EXPECT_FALSE(ism330dhcx_driver::accel_range_from_g(3).has_value());
  EXPECT_EQ(ism330dhcx_driver::gyro_range_from_dps(2000), GyroRange::Dps2000);
  EXPECT_FALSE(ism330dhcx_driver::gyro_range_from_dps(300).has_value());
}
