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

#ifndef ISM330DHCX_DRIVER__TESTING__FAKE_REGISTER_BUS_HPP_
#define ISM330DHCX_DRIVER__TESTING__FAKE_REGISTER_BUS_HPP_

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <system_error>
#include <utility>
#include <vector>

#include "ism330dhcx_driver/register_bus.hpp"
#include "ism330dhcx_driver/registers.hpp"

namespace ism330dhcx_driver::testing
{
// In-memory ISM330DHCX register file with a FIFO queue, for unit tests.
class FakeRegisterBus : public RegisterBus
{
public:
  using Word = std::array<uint8_t, reg::FIFO_WORD_SIZE>;

  FakeRegisterBus()
  {
    registers_.fill(0);
    registers_[reg::WHO_AM_I] = reg::WHO_AM_I_VALUE;
    registers_[reg::CTRL3_C] = reg::CTRL3_C_IF_INC;
    registers_[reg::CTRL9_XL] = 0xE0;
  }

  void read(uint8_t address, uint8_t * data, std::size_t len) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_failure();
    if (address == reg::FIFO_DATA_OUT_TAG && len == reg::FIFO_WORD_SIZE) {
      Word word{};
      if (!fifo_.empty()) {
        word = fifo_.front();
        fifo_.pop_front();
      }
      std::memcpy(data, word.data(), len);
      return;
    }
    if (address == reg::FIFO_STATUS1) {
      const auto level = static_cast<uint16_t>(fifo_.size());
      registers_[reg::FIFO_STATUS1] = static_cast<uint8_t>(level & 0xFF);
      registers_[reg::FIFO_STATUS2] = static_cast<uint8_t>(
        ((level >> 8) & reg::FIFO_STATUS2_DIFF_FIFO_HI_MASK) |
        (overrun_ ? reg::FIFO_STATUS2_OVR_IA : 0));
    }
    for (std::size_t i = 0; i < len; ++i) {
      data[i] = registers_[(address + i) & 0xFF];
    }
  }

  void write(uint8_t address, uint8_t value) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_failure();
    writes_.emplace_back(address, value);
    // The software reset completes instantly.
    registers_[address] = address == reg::CTRL3_C
                            ? static_cast<uint8_t>(value & ~reg::CTRL3_C_SW_RESET)
                            : value;
  }

  void push_word(uint8_t tag, int16_t x, int16_t y, int16_t z)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Word word{};
    word[0] = static_cast<uint8_t>(tag << reg::FIFO_TAG_SHIFT);
    const int16_t values[3] = {x, y, z};
    for (std::size_t i = 0; i < 3; ++i) {
      const auto raw = static_cast<uint16_t>(values[i]);
      word[1 + 2 * i] = static_cast<uint8_t>(raw & 0xFF);
      word[2 + 2 * i] = static_cast<uint8_t>(raw >> 8);
    }
    fifo_.push_back(word);
  }

  void push_gyro(int16_t x, int16_t y, int16_t z) { push_word(reg::FIFO_TAG_GYRO, x, y, z); }
  void push_accel(int16_t x, int16_t y, int16_t z) { push_word(reg::FIFO_TAG_ACCEL, x, y, z); }

  void set_register(uint8_t address, uint8_t value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    registers_[address] = value;
  }

  uint8_t get_register(uint8_t address)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return registers_[address];
  }

  void set_overrun(bool overrun)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    overrun_ = overrun;
  }

  void set_failing(bool failing)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    failing_ = failing;
  }

  std::size_t fifo_size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fifo_.size();
  }

  std::vector<std::pair<uint8_t, uint8_t>> writes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

private:
  void check_failure() const
  {
    if (failing_) {
      throw std::system_error(EIO, std::generic_category(), "fake bus failure");
    }
  }

  std::mutex mutex_;
  std::array<uint8_t, 256> registers_{};
  std::deque<Word> fifo_;
  std::vector<std::pair<uint8_t, uint8_t>> writes_;
  bool overrun_{false};
  bool failing_{false};
};
}  // namespace ism330dhcx_driver::testing

#endif  // ISM330DHCX_DRIVER__TESTING__FAKE_REGISTER_BUS_HPP_
