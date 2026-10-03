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

#include "ism330dhcx_driver/spi_bus.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <system_error>

#include "ism330dhcx_driver/registers.hpp"

namespace ism330dhcx_driver
{
namespace
{
// Transfers per SPI_IOC_MESSAGE ioctl; keeps the ioctl argument well below the 4 KiB page.
constexpr std::size_t MAX_TRANSFERS_PER_MESSAGE = 64;

[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}
}  // namespace

SpiBus::SpiBus(const std::string & device, uint32_t speed_hz) : speed_hz_(speed_hz)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw_errno("open " + device);
  }

  uint8_t mode = SPI_MODE_3;
  uint8_t bits = 8;
  if (
    ::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
    const int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(err, std::generic_category(), "configure " + device);
  }
}

SpiBus::~SpiBus()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void SpiBus::read(uint8_t reg, uint8_t * data, std::size_t len)
{
  tx_buffer_.assign(len + 1, 0);
  rx_buffer_.assign(len + 1, 0);
  tx_buffer_[0] = static_cast<uint8_t>(reg | reg::SPI_READ);

  spi_ioc_transfer xfer{};
  xfer.tx_buf = reinterpret_cast<uintptr_t>(tx_buffer_.data());
  xfer.rx_buf = reinterpret_cast<uintptr_t>(rx_buffer_.data());
  xfer.len = static_cast<uint32_t>(len + 1);
  xfer.speed_hz = speed_hz_;
  xfer.bits_per_word = 8;
  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) < 0) {
    throw_errno("spi read");
  }
  std::memcpy(data, rx_buffer_.data() + 1, len);
}

void SpiBus::write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {static_cast<uint8_t>(reg & ~reg::SPI_READ), value};
  spi_ioc_transfer xfer{};
  xfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
  xfer.len = sizeof(tx);
  xfer.speed_hz = speed_hz_;
  xfer.bits_per_word = 8;
  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) < 0) {
    throw_errno("spi write");
  }
}

void SpiBus::read_blocks(uint8_t reg, std::size_t block_len, std::size_t count, uint8_t * data)
{
  // One ioctl carries several transfers; cs_change toggles chip select between them so each
  // transfer restarts at `reg`. All transfers share the same (address + dummy bytes) tx buffer.
  const std::size_t xfer_len = block_len + 1;
  tx_buffer_.assign(xfer_len, 0);
  tx_buffer_[0] = static_cast<uint8_t>(reg | reg::SPI_READ);

  spi_ioc_transfer xfers[MAX_TRANSFERS_PER_MESSAGE];
  std::size_t done = 0;
  while (done < count) {
    const std::size_t n = std::min(count - done, MAX_TRANSFERS_PER_MESSAGE);
    rx_buffer_.assign(n * xfer_len, 0);
    std::memset(xfers, 0, sizeof(xfers));
    for (std::size_t i = 0; i < n; ++i) {
      xfers[i].tx_buf = reinterpret_cast<uintptr_t>(tx_buffer_.data());
      xfers[i].rx_buf = reinterpret_cast<uintptr_t>(rx_buffer_.data() + i * xfer_len);
      xfers[i].len = static_cast<uint32_t>(xfer_len);
      xfers[i].speed_hz = speed_hz_;
      xfers[i].bits_per_word = 8;
      xfers[i].cs_change = (i + 1 < n) ? 1 : 0;
    }
    // Same request code as SPI_IOC_MESSAGE(n), without its variable-length-array sizeof.
    const auto request = _IOC(_IOC_WRITE, SPI_IOC_MAGIC, 0, n * sizeof(spi_ioc_transfer));
    if (::ioctl(fd_, request, xfers) < 0) {
      throw_errno("spi read_blocks");
    }
    for (std::size_t i = 0; i < n; ++i) {
      std::memcpy(data + (done + i) * block_len, rx_buffer_.data() + i * xfer_len + 1, block_len);
    }
    done += n;
  }
}
}  // namespace ism330dhcx_driver
