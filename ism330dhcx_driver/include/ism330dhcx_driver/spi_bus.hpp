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

#ifndef ISM330DHCX_DRIVER__SPI_BUS_HPP_
#define ISM330DHCX_DRIVER__SPI_BUS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ism330dhcx_driver/register_bus.hpp"

namespace ism330dhcx_driver
{
// RegisterBus over a Linux spidev device (e.g. /dev/spidev0.0), SPI mode 3, 4-wire.
class SpiBus : public RegisterBus
{
public:
  static constexpr uint32_t DEFAULT_SPEED_HZ = 8'000'000;  // sensor max is 10 MHz

  explicit SpiBus(const std::string & device, uint32_t speed_hz = DEFAULT_SPEED_HZ);
  ~SpiBus() override;

  SpiBus(const SpiBus &) = delete;
  SpiBus & operator=(const SpiBus &) = delete;

  void read(uint8_t reg, uint8_t * data, std::size_t len) override;
  void write(uint8_t reg, uint8_t value) override;
  void read_blocks(uint8_t reg, std::size_t block_len, std::size_t count, uint8_t * data) override;

private:
  int fd_{-1};
  uint32_t speed_hz_;
  std::vector<uint8_t> tx_buffer_;
  std::vector<uint8_t> rx_buffer_;
};
}  // namespace ism330dhcx_driver

#endif  // ISM330DHCX_DRIVER__SPI_BUS_HPP_
