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

#ifndef ISM330DHCX_DRIVER__REGISTERS_HPP_
#define ISM330DHCX_DRIVER__REGISTERS_HPP_

#include <cstddef>
#include <cstdint>

// Register map of the ISM330DHCX (ST ISM330DHCX datasheet). Only what the driver uses.
namespace ism330dhcx_driver::reg
{
constexpr uint8_t FIFO_CTRL1 = 0x07;
constexpr uint8_t FIFO_CTRL2 = 0x08;
constexpr uint8_t FIFO_CTRL3 = 0x09;
constexpr uint8_t FIFO_CTRL4 = 0x0A;
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t CTRL4_C = 0x13;
constexpr uint8_t CTRL9_XL = 0x18;
constexpr uint8_t OUT_TEMP_L = 0x20;
constexpr uint8_t FIFO_STATUS1 = 0x3A;
constexpr uint8_t FIFO_STATUS2 = 0x3B;
constexpr uint8_t INTERNAL_FREQ_FINE = 0x63;
constexpr uint8_t FIFO_DATA_OUT_TAG = 0x78;

constexpr uint8_t WHO_AM_I_VALUE = 0x6B;

// CTRL3_C
constexpr uint8_t CTRL3_C_BDU = 0x40;
constexpr uint8_t CTRL3_C_IF_INC = 0x04;
constexpr uint8_t CTRL3_C_SW_RESET = 0x01;

// CTRL4_C
constexpr uint8_t CTRL4_C_I2C_DISABLE = 0x04;

// CTRL9_XL: datasheet recommends always setting DEVICE_CONF during configuration.
constexpr uint8_t CTRL9_XL_DEVICE_CONF = 0x02;

// CTRL2_G full-scale override bits
constexpr uint8_t CTRL2_G_FS_125 = 0x02;
constexpr uint8_t CTRL2_G_FS_4000 = 0x01;

// FIFO_CTRL4 FIFO_MODE
constexpr uint8_t FIFO_MODE_BYPASS = 0x00;
constexpr uint8_t FIFO_MODE_CONTINUOUS = 0x06;

// FIFO_STATUS2
constexpr uint8_t FIFO_STATUS2_WTM_IA = 0x80;
constexpr uint8_t FIFO_STATUS2_OVR_IA = 0x40;
constexpr uint8_t FIFO_STATUS2_FULL_IA = 0x20;
constexpr uint8_t FIFO_STATUS2_DIFF_FIFO_HI_MASK = 0x03;

// FIFO_DATA_OUT_TAG: TAG_SENSOR in bits [7:3]
constexpr uint8_t FIFO_TAG_SHIFT = 3;
constexpr uint8_t FIFO_TAG_GYRO = 0x01;
constexpr uint8_t FIFO_TAG_ACCEL = 0x02;

// Each FIFO word is one tag byte followed by six data bytes.
constexpr std::size_t FIFO_WORD_SIZE = 7;

// SPI: MSB of the address byte selects a read.
constexpr uint8_t SPI_READ = 0x80;
}  // namespace ism330dhcx_driver::reg

#endif  // ISM330DHCX_DRIVER__REGISTERS_HPP_
