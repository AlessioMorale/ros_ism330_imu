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

#ifndef ISM330DHCX_DRIVER__REGISTER_BUS_HPP_
#define ISM330DHCX_DRIVER__REGISTER_BUS_HPP_

#include <cstddef>
#include <cstdint>

namespace ism330dhcx_driver
{
// Register-level access to the sensor. Implementations throw std::system_error on bus failures.
class RegisterBus
{
public:
  virtual ~RegisterBus() = default;

  // Burst read of `len` bytes starting at `reg` (relies on CTRL3_C.IF_INC auto-increment).
  virtual void read(uint8_t reg, uint8_t * data, std::size_t len) = 0;
  virtual void write(uint8_t reg, uint8_t value) = 0;

  // Reads `count` independent bursts of `block_len` bytes, each starting again at `reg`.
  // Used to drain the FIFO one word per burst; buses can override it to batch the transfers.
  virtual void read_blocks(uint8_t reg, std::size_t block_len, std::size_t count, uint8_t * data)
  {
    for (std::size_t i = 0; i < count; ++i) {
      read(reg, data + i * block_len, block_len);
    }
  }
};
}  // namespace ism330dhcx_driver

#endif  // ISM330DHCX_DRIVER__REGISTER_BUS_HPP_
